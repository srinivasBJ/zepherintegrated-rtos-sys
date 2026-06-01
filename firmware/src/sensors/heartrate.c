/*
 * =============================================================================
 * MAX30102 Heart-Rate & SpO₂ Driver Implementation
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * Sensor: Maxim Integrated MAX30102
 *   - I²C address: 0x57
 *   - Integrated IR (880 nm) + Red (660 nm) LEDs and photodetector
 *   - Used for BPM via peak detection and SpO₂ via ratio-of-ratios
 *
 * Algorithm used here: simple peak-to-peak interval BPM.
 * Production systems should use Maxim's MAXREFDES117 reference algorithm.
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <math.h>
#include <string.h>

#include "sensors/heartrate.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(hr_drv, LOG_LEVEL_DBG);

/* ─── MAX30102 Register Addresses ──────────────────────────────────────── */
#define MAX30102_ADDR               0x57
#define REG_INTERRUPT_STATUS1       0x00
#define REG_INTERRUPT_STATUS2       0x01
#define REG_INTERRUPT_ENABLE1       0x02
#define REG_INTERRUPT_ENABLE2       0x03
#define REG_FIFO_WRITE_PTR          0x04
#define REG_OVERFLOW_CTR            0x05
#define REG_FIFO_READ_PTR           0x06
#define REG_FIFO_DATA               0x07
#define REG_FIFO_CONFIG             0x08
#define REG_MODE_CONFIG             0x09
#define REG_SPO2_CONFIG             0x0A
#define REG_LED1_PULSE_AMP          0x0C  /* Red  */
#define REG_LED2_PULSE_AMP          0x0D  /* IR   */
#define REG_PART_ID                 0xFF

#define MAX30102_PART_ID_VAL        0x15
#define MODE_HR_ONLY                0x02
#define MODE_SPO2                   0x03
#define MODE_MULTI_LED              0x07

/* ─── Simulated data ring buffer (used on native_posix) ────────────────── */

#define HR_BUF_SIZE 32
static uint32_t ir_buf[HR_BUF_SIZE];
static uint32_t red_buf[HR_BUF_SIZE];
static uint8_t  buf_head = 0;

/* ─── I²C device handle ─────────────────────────────────────────────────── */

static const struct device *i2c_dev;

/* ─── Helper: write single register ────────────────────────────────────── */
static int max30102_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write(i2c_dev, buf, 2, MAX30102_ADDR);
}

/* ─── Helper: read single register ─────────────────────────────────────── */
static int max30102_read_reg(uint8_t reg, uint8_t *out)
{
    return i2c_write_read(i2c_dev, MAX30102_ADDR, &reg, 1, out, 1);
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

int hr_init(void)
{
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I²C bus not ready");
        return -ENODEV;
    }

    /* Verify part ID */
    uint8_t part_id;
    int ret = max30102_read_reg(REG_PART_ID, &part_id);
    if (ret || part_id != MAX30102_PART_ID_VAL) {
        LOG_ERR("MAX30102 not found (part_id=0x%02X, ret=%d)", part_id, ret);
        return -ENODEV;
    }

    /* Reset */
    max30102_write_reg(REG_MODE_CONFIG, 0x40);
    k_msleep(100);

    /* SpO₂ mode, 100 SPS, 18-bit ADC */
    max30102_write_reg(REG_MODE_CONFIG,  MODE_SPO2);
    max30102_write_reg(REG_SPO2_CONFIG,  0x27);  /* 100 SPS, 411μs PW */
    max30102_write_reg(REG_LED1_PULSE_AMP, 0x24); /* Red  = 7.2 mA    */
    max30102_write_reg(REG_LED2_PULSE_AMP, 0x24); /* IR   = 7.2 mA    */
    max30102_write_reg(REG_FIFO_CONFIG,  0x4F);  /* 4-sample avg, FIFO rollover */
    max30102_write_reg(REG_INTERRUPT_ENABLE1, 0x40); /* PPG_RDY interrupt */

    memset(ir_buf, 0, sizeof(ir_buf));
    memset(red_buf, 0, sizeof(red_buf));

    LOG_INF("MAX30102 initialised — SpO₂ mode, 100 SPS");
    return 0;
}

/* ─── Peak-detection BPM algorithm ─────────────────────────────────────── */

/*
 * Simple threshold-crossing BPM:
 * Finds rising edges in the IR signal above a dynamic threshold.
 * Real implementation should use Maxim's reference algorithm.
 */
static uint16_t compute_bpm(const uint32_t *ir, uint8_t len)
{
    if (len < 4) return 70; /* default until buffer fills */

    uint32_t ir_max = 0, ir_min = UINT32_MAX;
    for (int i = 0; i < len; i++) {
        if (ir[i] > ir_max) ir_max = ir[i];
        if (ir[i] < ir_min) ir_min = ir[i];
    }

    uint32_t threshold = (ir_max + ir_min) / 2;
    int peaks = 0;
    bool above = false;

    for (int i = 0; i < len; i++) {
        if (!above && ir[i] > threshold) {
            peaks++;
            above = true;
        } else if (above && ir[i] <= threshold) {
            above = false;
        }
    }

    /* Buffer covers ~0.32 s at 100 SPS → scale to 60 s */
    float bpm = (float)peaks * (60.0f / ((float)len / 100.0f));
    return (uint16_t)CLAMP(bpm, 20, 250);
}

/* ─── SpO₂ ratio-of-ratios ──────────────────────────────────────────────── */

static uint8_t compute_spo2(const uint32_t *ir, const uint32_t *red, uint8_t len)
{
    if (len < 4) return 98; /* nominal until buffer fills */

    /* DC component */
    uint64_t ir_dc = 0, red_dc = 0;
    for (int i = 0; i < len; i++) { ir_dc += ir[i]; red_dc += red[i]; }
    ir_dc /= len; red_dc /= len;

    /* AC component (RMS) */
    float ir_ac = 0, red_ac = 0;
    for (int i = 0; i < len; i++) {
        float di = (float)ir[i]  - ir_dc;
        float dr = (float)red[i] - red_dc;
        ir_ac  += di * di;
        red_ac += dr * dr;
    }
    ir_ac  = sqrtf(ir_ac  / len);
    red_ac = sqrtf(red_ac / len);

    if (ir_dc == 0 || ir_ac < 1.0f) return 98;

    float r = (red_ac / (float)red_dc) / (ir_ac / (float)ir_dc);
    /* Linear approximation: SpO₂ = 104 - 17*R  (Maxim AN6409) */
    float spo2 = 104.0f - 17.0f * r;
    return (uint8_t)CLAMP((int)spo2, 70, 100);
}

/* ─── Read ──────────────────────────────────────────────────────────────── */

int hr_read(hr_sample_t *sample)
{
    if (!sample) return -EINVAL;

    /* Read FIFO — up to 32 samples available */
    uint8_t wr_ptr, rd_ptr;
    max30102_read_reg(REG_FIFO_WRITE_PTR, &wr_ptr);
    max30102_read_reg(REG_FIFO_READ_PTR,  &rd_ptr);

    uint8_t num_samples = (wr_ptr - rd_ptr + 32) % 32;
    if (num_samples == 0) {
        sample->valid = false;
        return -EAGAIN;
    }

    /* Read each 6-byte FIFO entry (3 bytes Red + 3 bytes IR) */
    uint8_t fifo_reg = REG_FIFO_DATA;
    for (int i = 0; i < num_samples && i < HR_BUF_SIZE; i++) {
        uint8_t raw[6];
        i2c_write_read(i2c_dev, MAX30102_ADDR, &fifo_reg, 1, raw, 6);

        uint32_t red_raw = ((uint32_t)(raw[0] & 0x03) << 16) |
                           ((uint32_t)raw[1] << 8) | raw[2];
        uint32_t ir_raw  = ((uint32_t)(raw[3] & 0x03) << 16) |
                           ((uint32_t)raw[4] << 8) | raw[5];

        red_buf[buf_head] = red_raw;
        ir_buf[buf_head]  = ir_raw;
        buf_head = (buf_head + 1) % HR_BUF_SIZE;
    }

    sample->bpm       = compute_bpm(ir_buf, HR_BUF_SIZE);
    sample->spo2      = compute_spo2(ir_buf, red_buf, HR_BUF_SIZE);
    sample->valid     = (ir_buf[0] > 50000); /* finger present threshold */
    sample->timestamp = k_uptime_get();

    return 0;
}

/* ─── Vital alert check ─────────────────────────────────────────────────── */

bool hr_vital_alert(const hr_sample_t *sample)
{
    if (!sample || !sample->valid) return false;
    return (sample->bpm < HR_MIN_BPM) || (sample->bpm > HR_MAX_BPM);
}

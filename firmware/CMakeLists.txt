# =============================================================================
# Vehicle Emergency Response System
# Zephyr RTOS CMakeLists.txt
# Compatible with: nrf52840dk, native_posix (simulation), qemu_cortex_m3
# =============================================================================

cmake_minimum_required(VERSION 3.20.0)

# Default board for development simulation
set(BOARD native_posix CACHE STRING "Target board")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(vehicle_emergency_response VERSION 1.0.0
        DESCRIPTION "Zephyr RTOS Vehicle Emergency Response System"
        LANGUAGES C)

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
target_sources(app PRIVATE
  # Entry point
  src/main.c

  # Sensor drivers
  src/sensors/accel.c
  src/sensors/heartrate.c
  src/sensors/gps.c

  # Decision / state machine
  src/decision/engine.c

  # Communication layer
  src/comm/gsm.c

  # Cloud HTTP client
  src/cloud/http_client.c
)

# POSIX simulation sources (native_posix only)
if(CONFIG_BOARD_NATIVE_POSIX)
  target_sources(app PRIVATE src/sim/sensor_sim.c)
endif()

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
target_include_directories(app PRIVATE
  src/
  src/sensors/
  src/decision/
  src/comm/
  src/cloud/
  include/
)

# ---------------------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------------------
target_compile_definitions(app PRIVATE
  CRASH_G_THRESHOLD=30      # 30g threshold for crash detection
  HR_MIN_BPM=40
  HR_MAX_BPM=180
  GPS_UART_BAUD=9600
  CLOUD_HOST="192.168.1.100"
  CLOUD_PORT=8000
  CLOUD_PATH="/api/event"
  EMERGENCY_SMS_NUMBER="+11234567890"
)

# ---------------------------------------------------------------------------
# Link libraries
# ---------------------------------------------------------------------------
target_link_libraries(app PRIVATE zephyr_interface)

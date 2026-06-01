/* Cloud HTTP Client Header */
#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include "vers_types.h"

/**
 * @brief POST an emergency event to the cloud backend.
 * @return 0 on success, negative errno on failure.
 */
int http_post_emergency(const emergency_event_t *event);

#endif /* HTTP_CLIENT_H */

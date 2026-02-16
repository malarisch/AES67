/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * REST API web server for the AES67 device.
 */

#ifndef WEBSERVER_H_
#define WEBSERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the HTTP server in the background.
 *
 * Registers the HTTP service and starts the Zephyr HTTP server
 * thread.  The server listens on port 80 for all interfaces.
 *
 * @return 0 on success, negative errno on error
 */
int webserver_start(void);

#ifdef __cplusplus
}
#endif

#endif /* WEBSERVER_H_ */

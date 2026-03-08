#pragma once

#include "driver/i2c.h"

/*
 * Per-port I2C bus mutex.
 *
 * Any driver that shares an I2C port with another driver must:
 *   1. Call i2c_bus_mutex_init(port) during its init (idempotent — safe
 *      to call more than once for the same port).
 *   2. Wrap every i2c_master_* transaction with i2c_bus_lock / unlock.
 *
 * This prevents bus corruption when two FreeRTOS tasks (e.g. oled_task
 * and the main-loop RTC sync) access the same I2C port concurrently.
 */

/**
 * Create the mutex for @p port if it does not already exist.
 * Must be called from a single task context (e.g. app_main) before
 * any tasks that use the port are started.
 */
void i2c_bus_mutex_init(i2c_port_t port);

/** Acquire the bus mutex for @p port.  Blocks indefinitely. */
void i2c_bus_lock(i2c_port_t port);

/** Release the bus mutex for @p port. */
void i2c_bus_unlock(i2c_port_t port);

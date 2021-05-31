/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for Keyboard scan matrix devices.
 * The scope of this API is simply to report which key event was triggered
 * and users can later decode keys using their desired scan code tables in
 * their application. In addition, typematic rate and delay can easily be
 * implemented using a timer if desired.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BBQ10KBD_H_
#define ZEPHYR_INCLUDE_DRIVERS_BBQ10KBD_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief KSCAN APIs
 * @defgroup keyboard_interface Keyboard Scan Driver APIs
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Keyboard scan callback called when user press/release
 * a key on a matrix keyboard.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param row Describes row change.
 * @param column Describes column change.
 * @param pressed Describes the kind of key event.
 */
typedef void (*bbq10kbd_callback_t)(const struct device *dev, char key,
				 bool pressed, bool hold);
/**
 * @cond INTERNAL_HIDDEN
 *
 * Keyboard scan driver API definition and system call entry points.
 *
 * (Internal use only.)
 */
typedef int (*bbq10kbd_config_t)(const struct device *dev,
			      bbq10kbd_callback_t callback);
typedef int (*bbq10kbd_set_backlight_t)(const struct device *dev,
			      uint8_t value);
typedef int (*bbq10kbd_disable_callback_t)(const struct device *dev);
typedef int (*bbq10kbd_enable_callback_t)(const struct device *dev);

__subsystem struct bbq10kbd_driver_api {
	bbq10kbd_config_t config;
	bbq10kbd_disable_callback_t disable_callback;
	bbq10kbd_enable_callback_t enable_callback;
	bbq10kbd_set_backlight_t set_backlight;
};
/**
 * @endcond
 */

/**
 * @brief Configure a Keyboard scan instance.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param callback called when keyboard devices reply to to a keyboard
 * event such as key pressed/released.
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int bbq10kbd_config(const struct device *dev,
			     bbq10kbd_callback_t callback);

static inline int z_impl_bbq10kbd_config(const struct device *dev,
					bbq10kbd_callback_t callback)
{
	const struct bbq10kbd_driver_api *api =
				(struct bbq10kbd_driver_api *)dev->api;

	return api->config(dev, callback);
}
/**
 * @brief Enables callback.
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int bbq10kbd_enable_callback(const struct device *dev);

static inline int z_impl_bbq10kbd_enable_callback(const struct device *dev)
{
	const struct bbq10kbd_driver_api *api =
			(const struct bbq10kbd_driver_api *)dev->api;

	if (api->enable_callback == NULL) {
		return -ENOTSUP;
	}

	return api->enable_callback(dev);
}

/**
 * @brief Disables callback.
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int bbq10kbd_disable_callback(const struct device *dev);

static inline int z_impl_bbq10kbd_disable_callback(const struct device *dev)
{
	const struct bbq10kbd_driver_api *api =
			(const struct bbq10kbd_driver_api *)dev->api;

	if (api->disable_callback == NULL) {
		return -ENOTSUP;
	}

	return api->disable_callback(dev);
}

/**
 * @brief Configure the Keyboard backlight.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param value backlight intensity.
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int bbq10kbd_set_backlight(const struct device *dev,
			     uint8_t value);

static inline int z_impl_bbq10kbd_set_backlight(const struct device *dev,
					uint8_t value)
{
	const struct bbq10kbd_driver_api *api =
				(struct bbq10kbd_driver_api *)dev->api;

	return api->set_backlight(dev, value);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/bbq10kbd_api.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_BBQ10KBD_H_ */

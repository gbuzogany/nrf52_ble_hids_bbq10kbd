#include <kernel.h>
#include "bbq10kbd_api.h"
#include <init.h>
#include <drivers/gpio.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "bbq10kbd.h"

#define DT_DRV_COMPAT solderparty_bbq10kbd

LOG_MODULE_REGISTER(BBQ10KBD, CONFIG_SENSOR_LOG_LEVEL);

struct bbq10kbd_data {
	const struct device *dev;
	const struct device *bus;
	const struct device *gpio;
	bbq10kbd_callback_t callback;
	struct gpio_callback gpio_cb;
	struct k_work work;
	bool enabled;
};

union bbq10kbd_bus_config {
	uint16_t i2c_addr;
};

struct bbq10kbd_config {
	const char *bus_label;
	const struct bbq10kbd_reg_io *reg_io;
	const union bbq10kbd_bus_config bus_config;
	const char *gpio_name;
	gpio_pin_t gpio_pin;
	gpio_dt_flags_t gpio_flags;
};

typedef int (*bbq10kbd_reg_read_fn)(const struct device *bus,
				  const union bbq10kbd_bus_config *bus_config,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*bbq10kbd_reg_write_fn)(const struct device *bus,
				   const union bbq10kbd_bus_config *bus_config,
				   uint8_t reg, uint8_t val);

struct bbq10kbd_reg_io {
	bbq10kbd_reg_read_fn read;
	bbq10kbd_reg_write_fn write;
};

static inline struct bbq10kbd_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline const struct bbq10kbd_config *to_config(const struct device *dev)
{
	return dev->config;
}

static inline const struct device *to_bus(const struct device *dev)
{
	return to_data(dev)->bus;
}

static inline const union bbq10kbd_bus_config *to_bus_config(const struct device *dev)
{
	return &to_config(dev)->bus_config;
}

static int bbq10kbd_reg_read_i2c(const struct device *bus,
			       const union bbq10kbd_bus_config *bus_config,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read(bus, bus_config->i2c_addr,
			      start, buf, size);
}

static int bbq10kbd_reg_write_i2c(const struct device *bus,
				const union bbq10kbd_bus_config *bus_config,
				uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(bus, bus_config->i2c_addr,
				  reg, val);
}

static const struct bbq10kbd_reg_io bbq10kbd_reg_io_i2c = {
	.read = bbq10kbd_reg_read_i2c,
	.write = bbq10kbd_reg_write_i2c,
};

static inline int bbq10kbd_reg_read(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size)
{
	return to_config(dev)->reg_io->read(to_bus(dev), to_bus_config(dev),
					    start, buf, size);
}

static inline int bbq10kbd_reg_write(const struct device *dev, uint8_t reg,
				   uint8_t val)
{
	return to_config(dev)->reg_io->write(to_bus(dev), to_bus_config(dev),
					     reg, val);
}

static int bbq10kbd_keyboard_enable_callback(const struct device *dev)
{
	struct bbq10kbd_data *data = dev->data;

	LOG_DBG("%s: enable cb", dev->name);
	data->enabled = true;
	return 0;
}

static int bbq10kbd_keyboard_disable_callback(const struct device *dev)
{
	struct bbq10kbd_data *data = dev->data;

	LOG_DBG("%s: disable cb", dev->name);
	data->enabled = false;
	return 0;
}


static int bbq10kbd_keyboard_configure(const struct device *dev, bbq10kbd_callback_t callback)
{
	struct bbq10kbd_data *data = dev->data;

	if (!callback) {
		LOG_ERR("Callback is null");
		return -EINVAL;
	}
	LOG_DBG("%s: set callback", dev->name);

	data->callback = callback;

	return 0;
}

static int bbq10kbd_keyboard_set_backlight(const struct device *dev,
			      uint8_t value)
{
	int err = bbq10kbd_reg_write(dev, _REG_BKL | _WRITE_MASK, value);
	if (err < 0) {
		printk("set_backlight failed: %d\n", err);
		return err;
	}
	return 0;
}

static const struct bbq10kbd_driver_api bbq10kbd_api_funcs = {
	.config = bbq10kbd_keyboard_configure,
	.enable_callback = bbq10kbd_keyboard_enable_callback,
	.disable_callback = bbq10kbd_keyboard_disable_callback,
	.set_backlight = bbq10kbd_keyboard_set_backlight,
};

static void bbq10kbd_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb,
				   uint32_t pin_mask)
{
	struct bbq10kbd_data *data =
		CONTAINER_OF(cb, struct bbq10kbd_data, gpio_cb);

	k_work_submit(&data->work);
}

static void bbq10kbd_handle_int(const struct device *dev)
{
	struct bbq10kbd_data *data = dev->data;

	uint8_t kbuf[2];
	int err;
		
	err = bbq10kbd_reg_read(dev, _REG_KEY,
				      &kbuf, 1);

	uint8_t count = kbuf[0] & KEY_COUNT_MASK;
	if (count > 0) {
		int err = bbq10kbd_reg_read(dev, _REG_FIF,
				      &kbuf, 2);

		switch (kbuf[0]) {
			case 0: // idle
				data->callback(dev, kbuf[1], false, false);
				break;
			case 1: // press
				data->callback(dev, kbuf[1], true, false);
				break;
			case 2: // long press
				data->callback(dev, kbuf[1], true, true);
				break;
			case 3: // release
				data->callback(dev, kbuf[1], false, false);
				break;
			default:
				break;
		}

		err = bbq10kbd_reg_write(dev, _REG_INT | _WRITE_MASK, 0x00);
	}
}

static void bbq10kbd_work_handler(struct k_work *work)
{
	struct bbq10kbd_data *data =
		CONTAINER_OF(work, struct bbq10kbd_data, work);

	bbq10kbd_handle_int(data->dev);
}

static int bbq10kbd_dev_init(const struct device *dev)
{
	struct bbq10kbd_data *data = to_data(dev);
	const struct bbq10kbd_config *config = to_config(dev);
	int err;

	uint8_t version;

	data->gpio = device_get_binding(config->gpio_name);
	if (data->gpio == NULL) {
		LOG_ERR("Could not find GPIO device");
		return -EINVAL;
	}

	// reset the keyboard
	err = bbq10kbd_reg_write(dev, _REG_RST, 0x00);
	if (err < 0) {
		printk("reset failed: %d", err);
		return err;
	}
	k_sleep(K_MSEC(100));

	// read version
	err = bbq10kbd_reg_read(dev, _REG_VER, &version, 1);
	if (err < 0) {
		printk("read version failed: %d", err);
		return err;
	}
	else {
		printk("Version: %d", version);
	}
	
	err = bbq10kbd_reg_write(dev, _REG_CFG | _WRITE_MASK, (CFG_OVERFLOW_INT | CFG_KEY_INT | CFG_REPORT_MODS));
	if (err < 0) {
		printk("configuration failed: %d", err);
		return err;
	}

	data->work.handler = bbq10kbd_work_handler;

	err = gpio_pin_configure(data->gpio, config->gpio_pin,
			   GPIO_INPUT | config->gpio_flags);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, config->gpio_name, config->gpio_pin);
		return;
	}

	err = gpio_pin_interrupt_configure(data->gpio,
					   config->gpio_pin,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, config->gpio_name, config->gpio_pin);
		return;
	}

	gpio_init_callback(&data->gpio_cb, bbq10kbd_gpio_callback,
			   BIT(config->gpio_pin));
	
	gpio_add_callback(data->gpio, &data->gpio_cb);

	return 0;
}

int bbq10kbd_init(const struct device *dev)
{
	const char *name = dev->name;
	struct bbq10kbd_data *data = to_data(dev);
	const struct bbq10kbd_config *config = to_config(dev);
	int rc;

	LOG_DBG("initializing %s", name);

	printk("irq_dev_name: %s\n", config->gpio_name);
	printk("irq_flags: %d\n", config->gpio_flags);
	printk("irq_pin: %d\n", config->gpio_pin);

	data->dev = dev;
	data->bus = device_get_binding(config->bus_label);
	if (!data->bus) {
		LOG_DBG("bus \"%s\" not found", config->bus_label);
		rc = -EINVAL;
		goto done;
	}

	rc = bbq10kbd_dev_init(dev);
	if (rc < 0) {
		rc = -EINVAL;
		goto done;
	}

	rc = 0;

done:
	if (rc == 0) {
		LOG_DBG("%s OK", name);
	} else {
		LOG_DBG("%s failed", name);
	}
	return rc;
}

/*
 * Device creation macro, shared by BBQ10KBD_DEFINE_SPI() and
 * BBQ10KBD_DEFINE_I2C().
 */

#define BBQ10KBD_DEVICE_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst,					\
			    bbq10kbd_init,				\
            	device_pm_control_nop,			\
			    &bbq10kbd_data_##inst,			\
			    &bbq10kbd_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &bbq10kbd_api_funcs);

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define BBQ10KBD_CONFIG(inst)						\
	{								\
		.bus_label = DT_INST_BUS_LABEL(inst),			\
		.reg_io = &bbq10kbd_reg_io_i2c,				\
		.bus_config =  { .i2c_addr = DT_INST_REG_ADDR(inst), },\
		.gpio_name = DT_INST_GPIO_LABEL(inst, int1_gpios),		\
		.gpio_pin = DT_INST_GPIO_PIN(inst, int1_gpios),		\
		.gpio_flags = DT_INST_GPIO_FLAGS(inst, int1_gpios), \
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define BBQ10KBD_DEFINE(inst)						\
	static struct bbq10kbd_data bbq10kbd_data_##inst;			\
	static const struct bbq10kbd_config bbq10kbd_config_##inst =	\
		BBQ10KBD_CONFIG(inst);				\
	BBQ10KBD_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(BBQ10KBD_DEFINE)

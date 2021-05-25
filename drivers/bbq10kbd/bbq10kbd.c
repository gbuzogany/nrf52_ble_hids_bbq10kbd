#include <kernel.h>
#include <init.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/kscan.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "bbq10kbd.h"

#define DT_DRV_COMPAT solderparty_bbq10kbd
#define BBQ10KBD_DRIVER_NAME	"bbq10kbd_driver"

LOG_MODULE_REGISTER(BBQ10KBD, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "bbq10kbd driver enabled without any devices"
#endif

struct bbq10kbd_data {
	const struct device *bus;

	/* Compensation parameters. */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;

	/* Compensated values. */
	int32_t comp_temp;
	uint32_t comp_press;
	uint32_t comp_humidity;

	/* Carryover between temperature and pressure/humidity compensation. */
	int32_t t_fine;

	uint8_t chip_id;
};

struct bbq10kbd_config {
	char *i2c_name;
	uint8_t i2c_addr;
    gpio_pin_t irq0_pin;
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

static int bbq10kbd_reg_read_i2c(const struct device *bus,
			       const uint8_t i2c_addr,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read(bus, i2c_addr,
			      start, buf, size);
}

static int bbq10kbd_reg_write_i2c(const struct device *bus,
				const uint8_t i2c_addr,
				uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(bus, i2c_addr,
				  reg, val);
}

static inline int bbq10kbd_reg_read(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size)
{
	return bbq10kbd_reg_read_i2c(to_bus(dev), (const uint8_t)&to_config(dev)->i2c_addr,
					    start, buf, size);
}

static inline int bbq10kbd_reg_write(const struct device *dev, uint8_t reg,
				   uint8_t val)
{
	return bbq10kbd_reg_write_i2c(to_bus(dev), (const uint8_t)&to_config(dev)->i2c_addr,
					     reg, val);
}

static int bbq10kbd_config(const struct device *dev)
{
    printk("BBQ10KBD config");
    return 0;
}

static int bbq10kbd_disable_callback(const struct device *dev)
{
    printk("BBQ10KBD disable");
    return 0;
}

static int bbq10kbd_enable_callback(const struct device *dev)
{
    printk("BBQ10KBD enable");
    return 0;
}

int bbq10kbd_init(const struct device *dev) 
{
    printk("BBQ10KBD init");
    return 0;
}

static const struct kscan_driver_api bbq10kbd_api_funcs = {
    .config = bbq10kbd_config,
    .disable_callback = bbq10kbd_disable_callback,
    .enable_callback = bbq10kbd_enable_callback,
};

#define BBQ10KBD_HAS_IRQ_IDX(inst, idx)					\
		DT_INST_PROP_HAS_IDX(inst, irq_gpios, idx)

#define BBQ10KBD_DEFINE(inst)						\
	static struct bbq10kbd_data bbq10kbd_data_##inst;			\
	static const struct bbq10kbd_config bbq10kbd_config_##inst =	\
	{								\
		.i2c_name = DT_INST_BUS_LABEL(inst),			       \
		.i2c_addr = DT_INST_REG_ADDR(inst),	    \
        .irq0_pin = COND_CODE_1(BBQ10KBD_HAS_IRQ_IDX(inst, 0),		\
            (DT_INST_GPIO_PIN_BY_IDX(inst, irq_gpios, 0)),		\
            (0)),						\
	}; \
    DEVICE_DT_INST_DEFINE(inst,					\
            bbq10kbd_init,				\
            device_pm_control_nop,			\
            &bbq10kbd_data_##inst,			\
            &bbq10kbd_config_##inst,			\
            POST_KERNEL,				\
            CONFIG_KSCAN_INIT_PRIORITY,		\
            &bbq10kbd_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(BBQ10KBD_DEFINE)


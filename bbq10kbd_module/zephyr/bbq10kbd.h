#ifndef ZEPHYR_DRIVERS_BBQ10KBD_H_
#define ZEPHYR_DRIVERS_BBQ10KBD_H_

#include <zephyr/types.h>
#include <device.h>

#define _REG_VER 1
#define _REG_CFG 2
#define _REG_INT 3
#define _REG_KEY 4
#define _REG_BKL 5
#define _REG_DEB 6
#define _REG_FRQ 7
#define _REG_RST 8
#define _REG_FIF 9

#define _WRITE_MASK (1 << 7)

#define CFG_OVERFLOW_ON  (1 << 0)
#define CFG_OVERFLOW_INT (1 << 1)
#define CFG_CAPSLOCK_INT (1 << 2)
#define CFG_NUMLOCK_INT  (1 << 3)
#define CFG_KEY_INT      (1 << 4)
#define CFG_PANIC_INT    (1 << 5)
#define CFG_REPORT_MODS  (1 << 6)

#define INT_OVERFLOW     (1 << 0)
#define INT_CAPSLOCK     (1 << 1)
#define INT_NUMLOCK      (1 << 2)
#define INT_KEY          (1 << 3)
#define INT_PANIC        (1 << 4)

#define KEY_CAPSLOCK     (1 << 5)
#define KEY_NUMLOCK      (1 << 6)
#define KEY_COUNT_MASK   (0x1F)

#endif /* ZEPHYR_DRIVERS_BBQ10KBD_H_ */
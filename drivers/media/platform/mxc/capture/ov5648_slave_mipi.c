/*
 * Copyright (C) 2015-2018 Glowforge, Inc. <opensource@glowforge.com>
 *
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

//~ #include <linux/module.h>
//~ #include <linux/i2c.h>
//~
//~ #include "v4l2-int-device.h"
//~ #include "mxc_v4l2_capture.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include "glowforge_priv_v4l2_def.h"

#define OV5648_SLAVE_CHIP_ID_HIGH_BYTE    0x300A
#define OV5648_SLAVE_CHIP_ID_LOW_BYTE     0x300B

static int ov5648_slave_probe(struct i2c_client *adapter,
                const struct i2c_device_id *device_id);
static int ov5648_slave_remove(struct i2c_client *client);

static struct sensor_data ov5648_data;
/*!
 * Maintains the information on the current state of the sesor.
 */

static const struct i2c_device_id ov5648_slave_id[] = {
    {"ov5648_mipi_slave", 0},
    {},
};


static struct i2c_driver ov5648_slave_i2c_driver = {
    .driver = {
          .owner = THIS_MODULE,
          .name  = "ov5648_mipi_slave",
          },
    .probe  = ov5648_slave_probe,
    .remove = ov5648_slave_remove,
    .id_table = ov5648_slave_id,
};


void ov5648_register_slave_i2c(struct i2c_client * i2c);

static s32 ov5648_slave_read_reg(u16 reg, u8 *val)
{
    struct sensor_data *sensor = &ov5648_data;
    struct i2c_client *client = sensor->i2c_client;
    struct i2c_msg msgs[2];
    u8 buf[2];
    int ret;

    buf[0] = reg >> 8;
    buf[1] = reg & 0xff;
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0) {
        pr_err("%s(mipi):reg=%x ret=%d\n", __func__, reg, ret);
        return ret;
    }
    *val = buf[0];
    pr_debug("%s(mipi):reg=%x,val=%x\n", __func__, reg, buf[0]);
    return buf[0];
}


/*!
 * ov5648 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5648_slave_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    u8 chip_id_high, chip_id_low;
    int retval;

    ov5648_data.i2c_client = client;


    retval = ov5648_slave_read_reg(OV5648_SLAVE_CHIP_ID_HIGH_BYTE, &chip_id_high);
    if (retval < 0 || chip_id_high != 0x56) {
        pr_warning("camera ov5648_mipi is not found\n");
        clk_disable_unprepare(ov5648_data.sensor_clk);
        return -ENODEV;
    }
    retval = ov5648_slave_read_reg(OV5648_SLAVE_CHIP_ID_LOW_BYTE, &chip_id_low);
    if (retval < 0 || chip_id_low != 0x48) {
        pr_warning("camera ov5648_mipi is not found\n");
        clk_disable_unprepare(ov5648_data.sensor_clk);
        return -ENODEV;
    }

    pr_info("ov5648_slave_probe\n");
    ov5648_register_slave_i2c(client);

    pr_info("camera ov5648_mipi_slave is found\n");
    return 0;
}

/*!
 * ov5648 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5648_slave_remove(struct i2c_client *client)
{
    return 0;
}

/*!
 * ov5648 init function
 * Called by insmod ov5648_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov5648_slave_init(void)
{
    u8 err;
    pr_info("Loading ov5648_mipi_slave driver\n");
    err = i2c_add_driver(&ov5648_slave_i2c_driver);
    if (err != 0)
        pr_err("%s:driver registration failed, error=%d\n",
            __func__, err);
    pr_info("Loaded ov5648_mipi_slave driver\n");
    return err;
}

/*!
 * OV5648 cleanup function
 * Called on rmmod ov5648_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov5648_slave_clean(void)
{
    i2c_del_driver(&ov5648_slave_i2c_driver);
}

module_init(ov5648_slave_init);
module_exit(ov5648_slave_clean);

MODULE_AUTHOR("Glowforge, Inc. <opensource@glowforge.com>");
MODULE_DESCRIPTION("OV5648 MIPI Slave Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

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

#define OV5648_NAME "ov5648_mipi"

/* For testing purposes, the OV5648 offers a test patterns
 * Uncomment APPLY_TEST_PATTERN to apply the pattern
 */
/*#define APPLY_TEST_PATTERN*/

#define OV5648_VOLTAGE_ANALOG               2800000
#define OV5648_VOLTAGE_DIGITAL_CORE         1500000
#define OV5648_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV5648_XCLK_MIN 6000000
#define OV5648_XCLK_MAX 27000000

#define OV5648_CHIP_ID_HIGH_BYTE    0x300A
#define OV5648_CHIP_ID_LOW_BYTE     0x300B

#define IO_STROBE_BIT (0x08)
#define IO_GPIO_BIT   (0x01)

/* ov5648 Register */

#define SC_CMMN_PAD_OEN2  (0x3002)
#define SC_CMMN_PAD_SEL2  (0x3010)
#define STROBE_RSTRB      (0x3B00)
#define ISP_CTRL3D        (0x503d)

#ifndef APPLY_TEST_PATTERN
#define ISP_CTRL3D_VALUE (0x00)
#else
#define ISP_CTRL3D_VALUE (0x80)
#endif


enum ov5648_mode {
    ov5648_mode_MIN = 0,
    ov5648_mode_960P_1280_960 = 0,
    ov5648_mode_720P_1280_720 = 1,
    ov5648_mode_1080P_1920_1080 = 2,
    ov5648_mode_VGA_640_480 = 3,
    ov5648_mode_2592_1944 = 4, /* ov5648_mode_XGA_1024_768 */
    ov5648_mode_960_720 = 5,
    ov5648_mode_VGA_640_480_narrow = 6,
    ov5648_mode_MAX = 6,
    ov5648_mode_INIT = 0xff, /*only for sensor init*/
};

enum ov5648_frame_rate {
    ov5648_15_fps,
    ov5648_30_fps
};

/* image size under 1280 * 960 are SUBSAMPLING
 * image size upper 1280 * 960 are SCALING
 */
enum ov5648_downsize_mode {
    SUBSAMPLING,
    SCALING,
};

enum ov5648_color {
    OV5648_RED,
    OV5648_GREEN,
    OV5648_BLUE,
};

struct reg_value {
    u16 u16RegAddr;
    u8 u8Val;
    u8 u8Mask;
    u32 u32Delay_ms;
};

struct ov5648_mode_info {
    enum ov5648_mode mode;
    enum ov5648_downsize_mode dn_mode;
    u32 width;
    u32 height;
    struct reg_value *init_data_ptr;
    u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ov5648_data;
static struct additional_data ov5648_data_add;
static int sel_gpio = -EINVAL;
static int sel_active;
static int strobe_software = 0;

#define INIT_ARRAY \
    {0x0100, 0x00, 0, 0}, /* Software Standy      */ \
    {0x0103, 0x01, 0, 0}, \
    {0x3001, 0x00, 0, 0}, \
    {0x3002, 0x00, 0, 0}, \
    {0x3011, 0x02, 0, 0}, \
    {0x3017, 0x05, 0, 0}, \
    {0x3018, 0x44, 0, 0}, /* was 4c */ \
    {0x301c, 0xd2, 0, 0}, \
    {0x3022, 0x00, 0, 0}, \
    {0x3034, 0x18, 0, 0}, /* 8-bit mode          */ \
    {0x3035, 0x21, 0, 0}, \
    {0x3036, 0x52, 0, 0}, /* MCLK=24MHz MIPI=420MBs*2 Lanes FPS=30 */ /* was 0x69 */ \
    {0x3037, 0x03, 0, 0}, \
    {0x3038, 0x00, 0, 0}, \
    {0x3039, 0x00, 0, 0}, \
    {0x303a, 0x00, 0, 0}, \
    {0x303b, 0x19, 0, 0}, \
    {0x303c, 0x11, 0, 0}, \
    {0x303d, 0x30, 0, 0}, \
    {0x3105, 0x11, 0, 0}, \
    {0x3106, 0x05, 0, 0}, \
    {0x3304, 0x28, 0, 0}, \
    {0x3305, 0x41, 0, 0}, \
    {0x3306, 0x30, 0, 0}, \
    {0x3308, 0x00, 0, 0}, \
    {0x3309, 0xc8, 0, 0}, \
    {0x330a, 0x01, 0, 0}, \
    {0x330b, 0x90, 0, 0}, \
    {0x330c, 0x02, 0, 0}, \
    {0x330d, 0x58, 0, 0}, \
    {0x330e, 0x03, 0, 0}, \
    {0x330f, 0x20, 0, 0}, \
    {0x3300, 0x00, 0, 0}, \
    {0x3500, 0x00, 0, 0}, \
    {0x3501, 0x7b, 0, 0}, \
    {0x3502, 0x00, 0, 0}, \
    {0x3503, 0x07, 0, 0}, \
    {0x350a, 0x00, 0, 0}, \
    {0x350b, 0x40, 0, 0}, \
    {0x3601, 0x33, 0, 0}, \
    {0x3602, 0x00, 0, 0}, \
    {0x3611, 0x0e, 0, 0}, \
    {0x3612, 0x2b, 0, 0}, \
    {0x3614, 0x50, 0, 0}, \
    {0x3620, 0x33, 0, 0}, \
    {0x3622, 0x00, 0, 0}, \
    {0x3630, 0xad, 0, 0}, \
    {0x3631, 0x00, 0, 0}, \
    {0x3632, 0x94, 0, 0}, \
    {0x3633, 0x17, 0, 0}, \
    {0x3634, 0x14, 0, 0}, \
    {0x3704, 0xc0, 0, 0}, \
    {0x3705, 0x2a, 0, 0}, \
    {0x3708, 0x63, 0, 0}, \
    {0x3709, 0x12, 0, 0}, \
    {0x370b, 0x23, 0, 0}, \
    {0x370c, 0xc0, 0, 0}, \
    {0x370d, 0x00, 0, 0}, \
    {0x370e, 0x00, 0, 0}, \
    {0x371c, 0x07, 0, 0}, \
    {0x3739, 0xd2, 0, 0}, \
    {0x373c, 0x00, 0, 0}, \
    {0x3800, 0x00, 0, 0}, \
    {0x3801, 0x00, 0, 0}, \
    {0x3802, 0x00, 0, 0}, \
    {0x3803, 0x00, 0, 0}, \
    {0x3804, 0x0a, 0, 0}, \
    {0x3805, 0x3f, 0, 0}, \
    {0x3806, 0x07, 0, 0}, \
    {0x3807, 0xa3, 0, 0}, \
    {0x3808, 0x0a, 0, 0}, \
    {0x3809, 0x20, 0, 0}, \
    {0x380a, 0x07, 0, 0}, \
    {0x380b, 0x98, 0, 0}, \
    {0x380c, 0x0b, 0, 0}, \
    {0x380d, 0x00, 0, 0}, \
    {0x380e, 0x07, 0, 0}, \
    {0x380f, 0xc0, 0, 0}, \
    {0x3810, 0x00, 0, 0}, \
    {0x3811, 0x10, 0, 0}, \
    {0x3812, 0x00, 0, 0}, \
    {0x3813, 0x06, 0, 0}, \
    {0x3814, 0x11, 0, 0}, \
    {0x3815, 0x11, 0, 0}, \
    {0x3817, 0x00, 0, 0}, \
    {0x3820, 0x40, 0, 0}, \
    {0x3821, 0x06, 0, 0}, \
    {0x3826, 0x03, 0, 0}, \
    {0x3829, 0x00, 0, 0}, \
    {0x382b, 0x0b, 0, 0}, \
    {0x3830, 0x00, 0, 0}, \
    {0x3836, 0x00, 0, 0}, \
    {0x3837, 0x00, 0, 0}, \
    {0x3838, 0x00, 0, 0}, \
    {0x3839, 0x04, 0, 0}, \
    {0x383a, 0x00, 0, 0}, \
    {0x383b, 0x01, 0, 0}, \
    {0X3A00, 0X58, 0, 0}, \
    {0x3b00, 0x00, 0, 0}, \
    {0x3b02, 0x08, 0, 0}, \
    {0x3b03, 0x00, 0, 0}, \
    {0x3b04, 0x04, 0, 0}, \
    {0x3b05, 0x00, 0, 0}, \
    {0x3b06, 0x04, 0, 0}, \
    {0x3b07, 0x08, 0, 0}, \
    {0x3b08, 0x00, 0, 0}, \
    {0x3b09, 0x02, 0, 0}, \
    {0x3b0a, 0x04, 0, 0}, \
    {0x3b0b, 0x00, 0, 0}, \
    {0x3b0c, 0x3d, 0, 0}, \
    {0x3f01, 0x0d, 0, 0}, \
    {0x3f0f, 0xf5, 0, 0}, \
    {0x4000, 0x89, 0, 0}, \
    {0x4001, 0x02, 0, 0}, \
    {0x4002, 0x45, 0, 0}, \
    {0x4004, 0x04, 0, 0}, \
    {0x4005, 0x18, 0, 0}, \
    {0x4006, 0x08, 0, 0}, \
    {0x4007, 0x10, 0, 0}, \
    {0x4008, 0x00, 0, 0}, \
    {0x4050, 0x6e, 0, 0}, \
    {0x4051, 0x8f, 0, 0}, \
    {0x4300, 0xf8, 0, 0}, \
    {0x4303, 0xff, 0, 0}, \
    {0x4304, 0x00, 0, 0}, \
    {0x4307, 0xff, 0, 0}, \
    {0x4520, 0x00, 0, 0}, \
    {0x4521, 0x00, 0, 0}, \
    {0x4511, 0x22, 0, 0}, \
    {0x4801, 0x0f, 0, 0}, \
    {0x4814, 0x2a, 0, 0}, \
    {0x481f, 0x3c, 0, 0}, \
    {0x4823, 0x3c, 0, 0}, \
    {0x4826, 0x00, 0, 0}, \
    {0x481b, 0x3c, 0, 0}, \
    {0x4827, 0x32, 0, 0}, \
    {0x4837, 0x24, 0, 0}, /* was 0x18 */ \
    {0x4b00, 0x06, 0, 0}, \
    {0x4b01, 0x0a, 0, 0}, \
    {0x4b04, 0x10, 0, 0}, \
    {0x5000, 0xff, 0, 0}, \
    {0x5001, 0x00, 0, 0}, \
    {0x5002, 0x41, 0, 0}, \
    {0x5003, 0x0a, 0, 0}, \
    {0x5004, 0x00, 0, 0}, \
    {0x5043, 0x00, 0, 0}, \
    {0x5013, 0x00, 0, 0}, \
    {0x501f, 0x03, 0, 0}, \
    {ISP_CTRL3D, ISP_CTRL3D_VALUE, 0, 0}, \
    {0x5a00, 0x08, 0, 0}, \
    {0x5b00, 0x01, 0, 0}, \
    {0x5b01, 0x40, 0, 0}, \
    {0x5b02, 0x00, 0, 0}, \
    {0x5b03, 0xf0, 0, 0},


static struct reg_value ov5648_setting_30fps_960P_1280_960[] = {
    INIT_ARRAY
    /* to 1280x960P30       */
    {0x301a, 0xf1, 0, 0}, /* DEBUG MODE */
    {0x3708, 0x66, 0, 0}, /* Analog Control Registers */
    {0x3709, 0x52, 0, 0}, /* Analog Control Registers */
    {0x370c, 0xc3, 0, 0}, /* Analog Control Registers */
    {0x3800, 0x00, 0, 0}, /* System timing register:  */
    {0x3801, 0x10, 0, 0}, /* X_ADDR_START = 16 (0x10) */
    {0x3802, 0x00, 0, 0}, /* System timing register: */
    {0x3803, 0x06, 0, 0}, /* Y_ADDR_START = 6 (0x06) */
    {0x3804, 0x0a, 0, 0}, /* System timing register:    */
    {0x3805, 0x2f, 0, 0}, /* X_ADDR_END = 2607 (0x0A2F) */
    {0x3806, 0x07, 0, 0}, /* System timing registers:   */
    {0x3807, 0x9d, 0, 0}, /* Y_ADDR_END = 1949 (0x079D) */
    {0x3808, 0x05, 0, 0}, /* Video output horizontal:      */
    {0x3809, 0x00, 0, 0}, /* X_OUTPUT_SIZE = 1280 (0x0500) */
    {0x380a, 0x03, 0, 0}, /* Video output vertical height: */
    {0x380b, 0xc0, 0, 0}, /* Y_OUTPUT_SIZE = 960 (0x3C0)   */
    {0x380c, 0x07, 0, 0}, /* Total horizontal size:    */
    {0x380d, 0x78, 0, 0}, /* TIMING_HTS = 1912 (0x778) */
    {0x380e, 0x05, 0, 0}, /* Total vertical size:      */
    {0x380f, 0xd8, 0, 0}, /* TIMING_VTS = 1496 (0x5d8) */
    {0x3810, 0x00, 0, 0}, /* ISP horizontal offset: */
    {0x3811, 0x08, 0, 0}, /* ISP_X_WIN = 8 (0x08)   */
    {0x3812, 0x00, 0, 0}, /* ISP vertical offset         */
    {0x3813, 0x06, 0, 0}, /* ISP_Y_WIN = 6 (0x06) */
    {0x3814, 0x31, 0, 0}, /* Horizontal subsample odd increase number: 0x3, Horizontal subsample even increase number 0x1 */
    {0x3815, 0x31, 0, 0}, /* Vertical subsample odd increase number 0x3, Vertical subsample even increase number 0x1 */
    {0x3820, 0x00, 0, 0}, /* TIMING_TC_REG20 */
    {0x3821, 0x07, 0, 0}, /* TIMING_TC_REG21 */
    {0x4004, 0x02, 0, 0}, /* DEBUG MODE: Black line num */
    {0x4005, 0x1a, 0, 0}, /* BLC CTRL05 */
    {0x301a, 0xf0, 0, 0}, /* DEBUG MODE */
};

static struct reg_value ov5648_setting_15fps_960P_1280_960[] = {
    {0x0, 0x0, 0, 0},
};


static struct reg_value ov5648_setting_30fps_720P_1280_720[] = {
    INIT_ARRAY
    /* to 1280x720P30 */
    {0x301a, 0xf1, 0, 0}, /* DEBUG MODE */
    {0x3708, 0x66, 0, 0}, /* Analog Control Registers */
    {0x3709, 0x52, 0, 0}, /* Analog Control Registers */
    {0x370c, 0xc3, 0, 0}, /* Analog Control Registers */
    {0x3800, 0x00, 0, 0}, /* System timing register:  */
    {0x3801, 0x10, 0, 0}, /* X_ADDR_START = 16 (0x10) */
    {0x3802, 0x00, 0, 0}, /* System timing register:   */
    {0x3803, 0xfe, 0, 0}, /* Y_ADDR_START = 254 (0xFE) */
    {0x3804, 0x0a, 0, 0}, /* System timing register:    */
    {0x3805, 0x2f, 0, 0}, /* X_ADDR_END = 2607 (0x0A2F) */
    {0x3806, 0x06, 0, 0}, /* System timing registers:   */
    {0x3807, 0xa5, 0, 0}, /* Y_ADDR_END = 1701 (0x06A5) */
    {0x3808, 0x05, 0, 0}, /* Video output horizontal: */
    {0x3809, 0x00, 0, 0}, /* X_OUTPUT_SIZE = 1280 (0x0500) */
    {0x380a, 0x02, 0, 0}, /* Video output vertical height: */
    {0x380b, 0xd0, 0, 0}, /* Y_OUTPUT_SIZE = 720 (0x2D0)   */
    {0x380c, 0x07, 0, 0}, /* Total horizontal size:     */
    {0x380d, 0x78, 0, 0}, /* TIMING_HTS = 1912 (0x1912) */
    {0x380e, 0x05, 0, 0}, /* Total vertical size:       */
    {0x380f, 0xd8, 0, 0}, /* TIMING_VTS = 1496 (0x5d8)  */
    {0x3810, 0x00, 0, 0}, /* ISP horizontal offset: */
    {0x3811, 0x08, 0, 0}, /* ISP_X_WIN = 8 0x08 */
    {0x3812, 0x00, 0, 0},
    {0x3813, 0x02, 0, 0}, /* ISP_Y_WIN = 2 (0x02) */
    {0x3814, 0x31, 0, 0},
    {0x3815, 0x31, 0, 0},
    {0x3820, 0x00, 0, 0},
    {0x3821, 0x07, 0, 0},
    {0x4004, 0x02, 0, 0},
    {0x4005, 0x1a, 0, 0},
    {0x301a, 0xf0, 0, 0},
};

static struct reg_value ov5648_setting_15fps_720P_1280_720[] = {
    {0x0, 0x0, 0, 0},
};

static struct reg_value ov5648_setting_30fps_1080P_1920_1080[] = {
    {0x0, 0x0, 0, 0},
};

static struct reg_value ov5648_setting_15fps_1080P_1920_1080[] = {
    {0x0, 0x0, 0, 0},
};
static struct reg_value ov5648_setting_30fps_VGA_640_480[] = {
    INIT_ARRAY
    /* to 640x480P30       */
    {0x301a, 0xf1, 0, 0}, /* DEBUG MODE */
    {0x3708, 0x66, 0, 0}, /* Analog Control Registers */
    {0x3709, 0x52, 0, 0}, /* Analog Control Registers */
    {0x370c, 0xc3, 0, 0}, /* Analog Control Registers */
    {0x3800, 0x00, 0, 0}, /* System timing register:  */
    {0x3801, 0x00, 0, 0}, /* X_ADDR_START = 16 (0x10) */
    {0x3802, 0x00, 0, 0}, /* System timing register: */
    {0x3803, 0x00, 0, 0}, /* Y_ADDR_START = 6 (0x06) */
    {0x3804, 0x0a, 0, 0}, /* System timing register:    */
    {0x3805, 0x3f, 0, 0}, /* X_ADDR_END = 2607 (0x0A2F) */
    {0x3806, 0x07, 0, 0}, /* System timing registers:   */
    {0x3807, 0xa1, 0, 0}, /* Y_ADDR_END = 1949 (0x079D) */
    {0x3808, 0x02, 0, 0}, /* Video output horizontal:      */
    {0x3809, 0x80, 0, 0}, /* X_OUTPUT_SIZE = 640 (0x0280) */
    {0x380a, 0x01, 0, 0}, /* Video output vertical height: */
    {0x380b, 0xE0, 0, 0}, /* Y_OUTPUT_SIZE = 480 (0x1E0)   */
    {0x380c, 0x07, 0, 0}, /* Total horizontal size:    */
    {0x380d, 0x68, 0, 0}, /* TIMING_HTS = 1912 (0x778) */
    {0x380e, 0x03, 0, 0}, /* Total vertical size:      */
    {0x380f, 0xd8, 0, 0}, /* TIMING_VTS = 1496 (0x5d8) */
    {0x3810, 0x00, 0, 0}, /* ISP horizontal offset: */
    {0x3811, 0x08, 0, 0}, /* ISP_X_WIN = 8 (0x08)   */
    {0x3812, 0x00, 0, 0}, /* ISP vertical offset         */
    {0x3813, 0x02, 0, 0}, /* TIMING_ISP_Y_WIN = 6 (0x06) */
    {0x3814, 0x71, 0, 0}, /* Horizontal subsample odd increase number: 0x3, Horizontal subsample even increase number 0x1 */
    {0x3815, 0x71, 0, 0}, /* Vertical subsample odd increase number 0x3, Vertical subsample even increase number 0x1 */
    {0x3820, 0x00, 0, 0}, /* TIMING_TC_REG20 */
    {0x3821, 0x07, 0, 0}, /* TIMING_TC_REG21 */
    {0x4004, 0x02, 0, 0}, /* DEBUG MODE: Black line num */
    {0x4005, 0x1a, 0, 0}, /* BLC CTRL05 */
    {0x301a, 0xf0, 0, 0}, /* DEBUG MODE */
};
static struct reg_value ov5648_setting_15fps_VGA_640_480[] = {
    {0x0, 0x0, 0, 0},
};
static struct reg_value ov5648_setting_30fps_XGA_1024_768[] = {
    {0x0, 0x0, 0, 0},
};

static struct reg_value ov5648_setting_15fps_2592_1944[] = {
    INIT_ARRAY
    /* to 2592x1944P15       */
    {0x301a, 0xf1},
    {0x3708, 0x63},
    {0x3709, 0x12},
    {0x370c, 0xc0},
    {0x3800, 0x00},
    {0x3801, 0x00}, /* X_ADDR_START = 0 (0x0) */
    {0x3802, 0x00},
    {0x3803, 0x00}, /* Y_ADDR_START = 0 (0x06) */
    {0x3804, 0x0a},
    {0x3805, 0x3f}, /* X_ADDR_END = 2623 (0x0A3F) */
    {0x3806, 0x07},
    {0x3807, 0xa3}, /* Y_ADDR_END = 1955 (0x07A3) */
    {0x3808, 0x0a},
    {0x3809, 0x20}, /* X_OUTPUT_SIZE = 2592 (0x0A20) */
    {0x380a, 0x07},
    {0x380b, 0x98}, /* Y_OUTPUT_SIZE = 1944 (0x798)   */
    {0x380c, 0x0b},
    {0x380d, 0x1c}, /* TIMING_HTS = 2844 (0xB1C) */
    {0x380e, 0x07},
    {0x380f, 0xb0}, /* TIMING_VTS = 1968 (0x7B0) */
    {0x3810, 0x00},
    {0x3811, 0x10}, /* ISP_X_WIN = 16 (0x10)   */
    {0x3812, 0x00},
    {0x3813, 0x06}, /* ISP_Y_WIN = 6 (0x06) */
    {0x3814, 0x11},
    {0x3815, 0x11},

    {0x3820, 0x00},
    {0x3821, 0x06},

    {0x4004, 0x04},
    {0x4005, 0x1a},
    {0x301a, 0xf0},
};

static struct reg_value ov5648_setting_30fps_960_720[] = {
    {0x0, 0x0, 0, 0},
};

static struct reg_value ov5648_setting_15fps_960_720[] = {
    {0x0, 0x0, 0, 0},
};

static struct ov5648_mode_info ov5648_mode_info_data[2][ov5648_mode_MAX + 1] = {
    {
        {ov5648_mode_960P_1280_960, SUBSAMPLING, 1280, 960,
        ov5648_setting_15fps_960P_1280_960,
        ARRAY_SIZE(ov5648_setting_15fps_960P_1280_960)},
        {ov5648_mode_720P_1280_720, SUBSAMPLING, 1280, 720,
        ov5648_setting_15fps_720P_1280_720,
        ARRAY_SIZE(ov5648_setting_15fps_720P_1280_720)},
        {ov5648_mode_1080P_1920_1080, SCALING, 1920, 1080,
        ov5648_setting_15fps_1080P_1920_1080,
        ARRAY_SIZE(ov5648_setting_15fps_1080P_1920_1080)},
        {ov5648_mode_VGA_640_480, SUBSAMPLING, 640, 480,
        ov5648_setting_15fps_VGA_640_480,
        ARRAY_SIZE(ov5648_setting_15fps_VGA_640_480)},

        {ov5648_mode_2592_1944, SUBSAMPLING, 2592, 1944,
        ov5648_setting_15fps_2592_1944,
        ARRAY_SIZE(ov5648_setting_15fps_2592_1944)},

        {ov5648_mode_960_720, SUBSAMPLING, 960, 720,
        ov5648_setting_15fps_960_720,
        ARRAY_SIZE(ov5648_setting_15fps_960_720)},
        {ov5648_mode_VGA_640_480_narrow, SUBSAMPLING, 640, 480,
        ov5648_setting_15fps_VGA_640_480,
        ARRAY_SIZE(ov5648_setting_15fps_VGA_640_480)},
    },
    {
        {ov5648_mode_960P_1280_960, SUBSAMPLING, 1280, 960,
        ov5648_setting_30fps_960P_1280_960,
        ARRAY_SIZE(ov5648_setting_30fps_960P_1280_960)},
        {ov5648_mode_720P_1280_720, SUBSAMPLING, 1280, 720,
        ov5648_setting_30fps_720P_1280_720,
        ARRAY_SIZE(ov5648_setting_30fps_720P_1280_720)},
        {ov5648_mode_1080P_1920_1080, SCALING, 1920, 1080,
        ov5648_setting_30fps_1080P_1920_1080,
        ARRAY_SIZE(ov5648_setting_30fps_1080P_1920_1080)},
        {ov5648_mode_VGA_640_480, SUBSAMPLING, 640, 480,
        ov5648_setting_30fps_VGA_640_480,
        ARRAY_SIZE(ov5648_setting_30fps_VGA_640_480)},

        {ov5648_mode_2592_1944, SUBSAMPLING, 1024, 768,
        ov5648_setting_30fps_XGA_1024_768,
        ARRAY_SIZE(ov5648_setting_30fps_XGA_1024_768)},

        {ov5648_mode_960_720, SUBSAMPLING, 960, 720,
        ov5648_setting_30fps_960_720,
        ARRAY_SIZE(ov5648_setting_15fps_960_720)},
        {ov5648_mode_VGA_640_480_narrow, SUBSAMPLING, 640, 480,
        ov5648_setting_30fps_VGA_640_480,
        ARRAY_SIZE(ov5648_setting_30fps_VGA_640_480)},
    },
};


static int ov5648_probe(struct i2c_client *adapter,
                const struct i2c_device_id *device_id);
static int ov5648_remove(struct i2c_client *client);

static s32 ov5648_read_reg(u16 reg, u8 *val);
static s32 ov5648_write_reg(u16 reg, u8 val);
static int ioctl_s_power(struct v4l2_int_device *s, int on);

static const struct i2c_device_id ov5648_id[] = {
    {OV5648_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, ov5648_id);

static struct i2c_driver ov5648_i2c_driver = {
    .driver = {
          .owner = THIS_MODULE,
          .name  = OV5648_NAME,
          },
    .probe  = ov5648_probe,
    .remove = ov5648_remove,
    .id_table = ov5648_id,
};

static struct i2c_client * i2c_client_slave = NULL;
static struct i2c_client * i2c_client_master = NULL;
static int camera_num = 0;


static void ov5648_standby(s32 enable)
{
    pr_debug("ov5648_mipi_camera_powerdown: powerdown=%x\n", enable);
    ov5648_write_reg(0x0100, (enable) ? 0x00 : 0x01);
}

static void ov5648_reset(void)
{

}

static int ov5648_power_on(struct device *dev)
{
    int ret = 0;
    return ret;
}

static s32 ov5648_write_reg(u16 reg, u8 val)
{
    u8 au8Buf[3] = {0};

    au8Buf[0] = reg >> 8;
    au8Buf[1] = reg & 0xff;
    au8Buf[2] = val;

    if (i2c_master_send(ov5648_data.i2c_client, au8Buf, 3) < 0) {
        pr_err("%s:write reg error:reg=%x,val=%x\n",
            __func__, reg, val);
        return -1;
    }
    pr_debug("reg=%x,val=%x\n", reg, val);
    return 0;
}

static s32 ov5648_read_reg(u16 reg, u8 *val)
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

static int prev_sysclk, prev_HTS;
static int AE_low, AE_high, AE_Target = 44;

static struct v4l2_int_device ov5648_int_device;

static void OV5648_stream_on(void)
{
    ioctl_s_power(&ov5648_int_device, 1);
    ov5648_write_reg(0x4202, 0x00);
}

static void OV5648_stream_off(void)
{
    ov5648_write_reg(0x4202, 0x0f);
    ioctl_s_power(&ov5648_int_device, 0);
}

static const int sclk_rdiv_map[] = {1, 2, 4, 8};

int OV5648_get_sysclk(void)
{
     /* calculate sysclk */
    int tmp;
    unsigned Multiplier, PreDiv, SysDiv, Pll_rdiv, Bit_div2x = 1;
    unsigned div, sclk_rdiv, sysclk;
    u8 temp;

    tmp = ov5648_read_reg(0x3034, &temp);
    if (tmp < 0)
        return tmp;
    tmp &= 0x0f;
    if (tmp == 8 || tmp == 10)
        Bit_div2x = tmp / 2;

    tmp = ov5648_read_reg(0x3035, &temp);
    if (tmp < 0)
        return tmp;
    SysDiv = tmp >> 4;
    if (SysDiv == 0)
           SysDiv = 16;

    tmp = ov5648_read_reg(0x3036, &temp);
    if (tmp < 0)
        return tmp;
    Multiplier = tmp;

    tmp = ov5648_read_reg(0x3037, &temp);
    if (tmp < 0)
        return tmp;
    PreDiv = tmp & 0x0f;
    Pll_rdiv = ((tmp >> 4) & 0x01) + 1;

    tmp = ov5648_read_reg(0x3108, &temp);
    if (tmp < 0)
        return tmp;
    sclk_rdiv = sclk_rdiv_map[tmp & 0x03];

    sysclk = ov5648_data.mclk / 10000 * Multiplier;
    div = PreDiv * SysDiv * Pll_rdiv * Bit_div2x * sclk_rdiv;
    if (!div) {
        pr_err("%s:Error divide by 0, (%d * %d * %d * %d * %d)\n",
            __func__, PreDiv, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv);
        return -EINVAL;
    }
    if (!sysclk) {
        pr_err("%s:Error 0 clk, ov5648_data.mclk=%d, Multiplier=%d\n",
            __func__, ov5648_data.mclk, Multiplier);
        return -EINVAL;
    }
    sysclk /= div;
    pr_debug("%s: sysclk(%d) = %d / 10000 * %d / (%d * %d * %d * %d * %d)\n",
        __func__, sysclk, ov5648_data.mclk, Multiplier,
        PreDiv, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv);
    return sysclk;
}

void OV5648_set_night_mode(void)
{
     /* read HTS from register settings */
    u8 mode;

    ov5648_read_reg(0x3a00, &mode);
    mode &= 0xfb;
    ov5648_write_reg(0x3a00, mode);
}

int OV5648_get_HTS(void)
{
     /* read HTS from register settings */
    int HTS;
    u8 temp;

    HTS = ov5648_read_reg(0x380c, &temp);
    HTS = (HTS<<8) + ov5648_read_reg(0x380d, &temp);

    return HTS;
}

int OV5648_get_VTS(void)
{
     /* read VTS from register settings */
    int VTS;
    u8 temp;

    /* total vertical size[15:8] high byte */
    VTS = ov5648_read_reg(0x380e, &temp);

    VTS = (VTS<<8) + ov5648_read_reg(0x380f, &temp);

    return VTS;
}

int OV5648_set_VTS(int VTS)
{
     /* write VTS to registers */
     int temp;

     temp = VTS & 0xff;
     ov5648_write_reg(0x380f, temp);

     temp = VTS>>8;
     ov5648_write_reg(0x380e, temp);

     return 0;
}

int OV5648_get_shutter(void)
{
     /* read shutter, in number of line period */
    int shutter;
    u8 temp;

    shutter = (ov5648_read_reg(0x03500, &temp) & 0x0f);
    shutter = (shutter<<8) + ov5648_read_reg(0x3501, &temp);
    shutter = (shutter<<4) + (ov5648_read_reg(0x3502, &temp)>>4);

     return shutter;
}

int OV5648_set_shutter(int shutter)
{
     /* write shutter, in number of line period */
     int temp;

     shutter = shutter & 0xffff;

     temp = shutter & 0x0f;
     temp = temp<<4;
     ov5648_write_reg(0x3502, temp);

     temp = shutter & 0xfff;
     temp = temp>>4;
     ov5648_write_reg(0x3501, temp);

     temp = shutter>>12;
     ov5648_write_reg(0x3500, temp);

     return 0;
}

int OV5648_get_gain16(void)
{
     /* read gain, 16 = 1x */
    int gain16;
    u8 temp;

    gain16 = ov5648_read_reg(0x350a, &temp) & 0x03;
    gain16 = (gain16<<8) + ov5648_read_reg(0x350b, &temp);

    return gain16;
}

int OV5648_set_gain16(int gain16)
{
    /* write gain, 16 = 1x */
    u8 temp;
    gain16 = gain16 & 0x3ff;

    temp = gain16 & 0xff;
    ov5648_write_reg(0x350b, temp);

    temp = gain16>>8;
    ov5648_write_reg(0x350a, temp);

    return 0;
}

void OV5648_set_color_gain(int gain16, int color)
{
    u8 temp;

    if (gain16 >= 0)
    {
        /* Output manual gains set by registers */
        ov5648_read_reg(0x5180, &temp);
        temp = temp | 0x08;
        ov5648_write_reg(0x5180, temp);
    }
    else
    {
        /* Output calculated gains */
        ov5648_read_reg(0x5180, &temp);
        temp = temp & ~(0x08);
        ov5648_write_reg(0x5180, temp);
        return;
    }
    /* Limit the data */
    gain16 = gain16 & 0xFFF;

    switch (color)
    {
        case OV5648_RED:
            /* Set MANUAL RED GAIN LSB */
            temp = gain16 & 0xFF;
            ov5648_write_reg(0x5187, temp);
            /* Set MANUAL RED GAIN MSB */
            temp = gain16 >> 8;
            ov5648_write_reg(0x5186, temp);
            break;
        case OV5648_BLUE:
            /* Set MANUAL RED GAIN LSB */
            temp = gain16 & 0xFF;
            ov5648_write_reg(0x518B, temp);
            /* Set MANUAL RED GAIN MSB */
            temp = gain16 >> 8;
            ov5648_write_reg(0x518A, temp);
            break;
        case OV5648_GREEN:
            /* Set MANUAL RED GAIN LSB */
            temp = gain16 & 0xFF;
            ov5648_write_reg(0x5189, temp);
            /* Set MANUAL RED GAIN MSB */
            temp = gain16 >> 8;
            ov5648_write_reg(0x5188, temp);
            break;
    }
    return;
}

int OV5648_get_color_gain(int color)
{
    int gain16;
    u8 temp;

    switch (color)
    {
        case OV5648_RED:
            /* Get MANUAL RED GAIN MSB */
            gain16 = ov5648_read_reg(0x5186, &temp) & 0x0F;
            /* Get MANUAL RED GAIN LSB */
            gain16 = (gain16<<8) + ov5648_read_reg(0x5187, &temp);
            return gain16;
        case OV5648_BLUE:
            /* Get MANUAL RED GAIN MSB */
            gain16 = ov5648_read_reg(0x518A, &temp) & 0x0F;
            /* Get MANUAL RED GAIN LSB */
            gain16 = (gain16<<8) + ov5648_read_reg(0x518B, &temp);
            return gain16;
        case OV5648_GREEN:
            /* Get MANUAL RED GAIN MSB */
            gain16 = ov5648_read_reg(0x5188, &temp) & 0x0F;
            /* Get MANUAL RED GAIN LSB */
            gain16 = (gain16<<8) + ov5648_read_reg(0x5189, &temp);
            return gain16;
    }
    return -1;
}

int OV5648_get_light_freq(void)
{
    /* get banding filter value */
    int temp, temp1, light_freq = 0;
    u8 tmp;

    temp = ov5648_read_reg(0x3c01, &tmp);

    if (temp & 0x80) {
        /* manual */
        temp1 = ov5648_read_reg(0x3c00, &tmp);
        if (temp1 & 0x04) {
            /* 50Hz */
            light_freq = 50;
        } else {
            /* 60Hz */
            light_freq = 60;
        }
    } else {
        /* auto */
        temp1 = ov5648_read_reg(0x3c0c, &tmp);
        if (temp1 & 0x01) {
            /* 50Hz */
            light_freq = 50;
        } else {
            /* 60Hz */
            light_freq = 60;
        }
    }
    return light_freq;
}

void OV5648_set_bandingfilter(void)
{
    int prev_VTS;
    int band_step60, max_band60, band_step50, max_band50;

    /* read preview PCLK */
    prev_sysclk = OV5648_get_sysclk();
    /* read preview HTS */
    prev_HTS = OV5648_get_HTS();

    /* read preview VTS */
    prev_VTS = OV5648_get_VTS();

    /* calculate banding filter */
    /* 60Hz */
    band_step60 = prev_sysclk * 100/prev_HTS * 100/120;
    ov5648_write_reg(0x3a0a, (band_step60 >> 8));
    ov5648_write_reg(0x3a0b, (band_step60 & 0xff));

    max_band60 = (int)((prev_VTS-4)/band_step60);
    ov5648_write_reg(0x3a0d, max_band60);

    /* 50Hz */
    band_step50 = prev_sysclk * 100/prev_HTS;
    ov5648_write_reg(0x3a08, (band_step50 >> 8));
    ov5648_write_reg(0x3a09, (band_step50 & 0xff));

    max_band50 = (int)((prev_VTS-4)/band_step50);
    ov5648_write_reg(0x3a0e, max_band50);
}

int OV5648_set_AE_target(int target)
{
    /* stable in high */
    int fast_high, fast_low;
    AE_low = target * 23 / 25;  /* 0.92 */
    AE_high = target * 27 / 25; /* 1.08 */

    fast_high = AE_high<<1;
    if (fast_high > 255)
        fast_high = 255;

    fast_low = AE_low >> 1;

    ov5648_write_reg(0x3a0f, AE_high);
    ov5648_write_reg(0x3a10, AE_low);
    ov5648_write_reg(0x3a1b, AE_high);
    ov5648_write_reg(0x3a1e, AE_low);
    ov5648_write_reg(0x3a11, fast_high);
    ov5648_write_reg(0x3a1f, fast_low);

    return 0;
}

void OV5648_turn_on_AE_AG(int enable)
{
    u8 ae_ag_ctrl;

    ov5648_read_reg(0x3503, &ae_ag_ctrl);
    if (enable) {
        /* turn on auto AE/AG */
        ae_ag_ctrl = ae_ag_ctrl & ~(0x03);
    } else {
        /* turn off AE/AG */
        ae_ag_ctrl = ae_ag_ctrl | 0x03;
    }
    ov5648_write_reg(0x3503, ae_ag_ctrl);
}

bool binning_on(void)
{
    u8 temp;
    ov5648_read_reg(0x3821, &temp);
    temp &= 0xfe;
    if (temp)
        return true;
    else
        return false;
}

static void ov5648_set_virtual_channel(int channel)
{
    u8 channel_id;

    ov5648_read_reg(0x4814, &channel_id);
    channel_id &= ~(3 << 6);
    ov5648_write_reg(0x4814, channel_id | (channel << 6));
}

/* download ov5648 settings to sensor through i2c */
static int ov5648_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
    register u32 Delay_ms = 0;
    register u16 RegAddr = 0;
    register u8 Mask = 0;
    register u8 Val = 0;
    u8 RegVal = 0;
    int i, retval = 0;

    for (i = 0; i < ArySize; ++i, ++pModeSetting) {
        Delay_ms = pModeSetting->u32Delay_ms;
        RegAddr = pModeSetting->u16RegAddr;
        Val = pModeSetting->u8Val;
        Mask = pModeSetting->u8Mask;

        if (Mask) {
            retval = ov5648_read_reg(RegAddr, &RegVal);
            if (retval < 0)
                goto err;

            RegVal &= ~(u8)Mask;
            Val &= Mask;
            Val |= RegVal;
        }

        retval = ov5648_write_reg(RegAddr, Val);
        if (retval < 0)
            goto err;

        if (Delay_ms)
            msleep(Delay_ms);
    }
err:
    return retval;
}

/* sensor changes between scaling and subsampling
 * go through exposure calcualtion
 */
static int ov5648_change_mode_exposure_calc(enum ov5648_frame_rate frame_rate,
                enum ov5648_mode mode)
{
    struct reg_value *pModeSetting = NULL;
    s32 ArySize = 0;
    u8 average;
    int prev_shutter, prev_gain16;
    int cap_shutter, cap_gain16;
    int cap_sysclk, cap_HTS, cap_VTS;
    int light_freq, cap_bandfilt, cap_maxband;
    long cap_gain16_shutter;
    int retval = 0;

    /* check if the input mode and frame rate is valid */
    pModeSetting =
        ov5648_mode_info_data[frame_rate][mode].init_data_ptr;
    ArySize =
        ov5648_mode_info_data[frame_rate][mode].init_data_size;

    ov5648_data.pix.width =
        ov5648_mode_info_data[frame_rate][mode].width;
    ov5648_data.pix.height =
        ov5648_mode_info_data[frame_rate][mode].height;
    ov5648_data_add.map_sizeimage =
        ov5648_data.pix.width * ov5648_data.pix.height * 3 / 2;

    if (ov5648_data.pix.width == 0 || ov5648_data.pix.height == 0 ||
        pModeSetting == NULL || ArySize == 0)
        return -EINVAL;

    /* turn off AE/AG */
    OV5648_turn_on_AE_AG(0);

    /* read preview shutter */
    prev_shutter = OV5648_get_shutter();
    if ((binning_on()) && (mode != ov5648_mode_960P_1280_960) &&
        (mode != ov5648_mode_720P_1280_720) && (mode != ov5648_mode_1080P_1920_1080))
        prev_shutter *= 2;

    /* read preview gain */
    prev_gain16 = OV5648_get_gain16();

    /* get average */
    ov5648_read_reg(0x5693, &average);

    /* turn off night mode for capture */
    OV5648_set_night_mode();

    /* turn off overlay */
    /* ov5648_write_reg(0x3022, 0x06);//if no af function, just skip it */

    OV5648_stream_off();

    /* Write capture setting */
    retval = ov5648_download_firmware(pModeSetting, ArySize);
    if (retval < 0)
        goto err;

    /* read capture VTS */
    cap_VTS = OV5648_get_VTS();
    cap_HTS = OV5648_get_HTS();
    cap_sysclk = OV5648_get_sysclk();

    /* calculate capture banding filter */
    light_freq = OV5648_get_light_freq();
    if (light_freq == 60) {
        /* 60Hz */
        cap_bandfilt = cap_sysclk * 100 / cap_HTS * 100 / 120;
    } else {
        /* 50Hz */
        cap_bandfilt = cap_sysclk * 100 / cap_HTS;
    }
    cap_maxband = (int)((cap_VTS - 4)/cap_bandfilt);

    /* calculate capture shutter/gain16 */
    if (average > AE_low && average < AE_high) {
        /* in stable range */
        cap_gain16_shutter =
          prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
          * prev_HTS/cap_HTS * AE_Target / average;
    } else {
        cap_gain16_shutter =
          prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
          * prev_HTS/cap_HTS;
    }

    /* gain to shutter */
    if (cap_gain16_shutter < (cap_bandfilt * 16)) {
        /* shutter < 1/100 */
        cap_shutter = cap_gain16_shutter/16;
        if (cap_shutter < 1)
            cap_shutter = 1;

        cap_gain16 = cap_gain16_shutter/cap_shutter;
        if (cap_gain16 < 16)
            cap_gain16 = 16;
    } else {
        if (cap_gain16_shutter >
                (cap_bandfilt * cap_maxband * 16)) {
            /* exposure reach max */
            cap_shutter = cap_bandfilt * cap_maxband;
            cap_gain16 = cap_gain16_shutter / cap_shutter;
        } else {
            /* 1/100 < (cap_shutter = n/100) =< max */
            cap_shutter =
              ((int) (cap_gain16_shutter/16 / cap_bandfilt))
              *cap_bandfilt;
            cap_gain16 = cap_gain16_shutter / cap_shutter;
        }
    }

    /* write capture gain */
    OV5648_set_gain16(cap_gain16);

    /* write capture shutter */
    if (cap_shutter > (cap_VTS - 4)) {
        cap_VTS = cap_shutter + 4;
        OV5648_set_VTS(cap_VTS);
    }
    OV5648_set_shutter(cap_shutter);

    OV5648_stream_on();

err:
    return retval;
}

/* if sensor changes inside scaling or subsampling
 * change mode directly
 * */
static int ov5648_change_mode_direct(enum ov5648_frame_rate frame_rate,
                enum ov5648_mode mode)
{
    struct reg_value *pModeSetting = NULL;
    s32 ArySize = 0;
    int retval = 0;

    /* check if the input mode and frame rate is valid */
    pModeSetting =
        ov5648_mode_info_data[frame_rate][mode].init_data_ptr;
    ArySize =
        ov5648_mode_info_data[frame_rate][mode].init_data_size;

    ov5648_data.pix.width =
        ov5648_mode_info_data[frame_rate][mode].width;
    ov5648_data.pix.height =
        ov5648_mode_info_data[frame_rate][mode].height;
    ov5648_data_add.map_sizeimage =
        ov5648_data.pix.width * ov5648_data.pix.height * 3 / 2;

    if (ov5648_data.pix.width == 0 || ov5648_data.pix.height == 0 ||
        pModeSetting == NULL || ArySize == 0)
        return -EINVAL;

    /* turn off AE/AG */
    OV5648_turn_on_AE_AG(0);

    OV5648_stream_off();

    /* Write capture setting */
    retval = ov5648_download_firmware(pModeSetting, ArySize);
    if (retval < 0)
        goto err;

    OV5648_stream_on();

    OV5648_turn_on_AE_AG(1);

err:
    return retval;
}

static int ov5648_init_mode(enum ov5648_frame_rate frame_rate,
                enum ov5648_mode mode, enum ov5648_mode orig_mode)
{
    struct reg_value *pModeSetting = NULL;
    s32 ArySize = 0;
    int retval = 0;
    void *mipi_csi2_info;
    u32 mipi_reg, msec_wait4stable = 0;
    enum ov5648_downsize_mode dn_mode, orig_dn_mode;

    if ((mode > ov5648_mode_MAX || mode < ov5648_mode_MIN)
        && (mode != ov5648_mode_INIT)) {
        pr_err("Wrong ov5648 mode detected!\n");
        return -1;
    }

    mipi_csi2_info = mipi_csi2_get_info();

    /* initial mipi dphy */
    if (!mipi_csi2_info) {
        printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
               __func__, __FILE__);
        return -1;
    }

    ov5648_write_reg(0x4800, 0x25);
    OV5648_stream_off();

    if (!mipi_csi2_get_status(mipi_csi2_info))
        mipi_csi2_enable(mipi_csi2_info);

    if (!mipi_csi2_get_status(mipi_csi2_info)) {
        pr_err("Can not enable mipi csi2 driver!\n");
        return -1;
    }

    mipi_csi2_set_lanes(mipi_csi2_info);

    /*Only reset MIPI CSI2 HW at sensor initialize*/
    if (mode == ov5648_mode_INIT)
    {
        mipi_csi2_reset(mipi_csi2_info);
    }

    /* reg 0x3034[3:0] == 0x8 is 8bit mode */
    mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RAW8);

    if (orig_mode != ov5648_mode_INIT) {
        dn_mode = ov5648_mode_info_data[frame_rate][mode].dn_mode;
        orig_dn_mode = ov5648_mode_info_data[frame_rate][orig_mode].dn_mode;
    }
    else {
        orig_dn_mode = dn_mode = 0;
    }

    if (mode == ov5648_mode_INIT) {
        int index = (int)ov5648_data.streamcap.capturemode;

        pModeSetting = ov5648_mode_info_data[frame_rate][index].init_data_ptr;
        ArySize = ov5648_mode_info_data[frame_rate][index].init_data_size;

        retval = ov5648_download_firmware(pModeSetting, ArySize);
        if (retval < 0)
            goto err;
    } else if ((dn_mode == SUBSAMPLING && orig_dn_mode == SCALING) ||
            (dn_mode == SCALING && orig_dn_mode == SUBSAMPLING)) {
        /* change between subsampling and scaling
         * go through exposure calucation */
        retval = ov5648_change_mode_exposure_calc(frame_rate, mode);
    } else {
        /* change inside subsampling or scaling
         * download firmware directly */
        retval = ov5648_change_mode_direct(frame_rate, mode);
    }

    if (retval < 0)
        goto err;

    OV5648_set_AE_target(AE_Target);
    OV5648_get_light_freq();
    OV5648_set_bandingfilter();
    ov5648_set_virtual_channel(ov5648_data.virtual_channel);

    /* add delay to wait for sensor stable */
    if (frame_rate == ov5648_15_fps) {
        /* dump the first nine frames: 1/15*9 */
        msec_wait4stable = 600;
    } else if (frame_rate == ov5648_30_fps) {
        /* dump the first nine frames: 1/30*9 */
        msec_wait4stable = 300;
    }
    msleep(msec_wait4stable);

    if (mipi_csi2_info) {
        unsigned int i = 0;
        u8 resetval;

        /* wait for mipi sensor ready */
        while (1) {
            mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
            if (mipi_reg != 0x200)
                break;
            if (i++ >= 20) {
                pr_err("mipi csi2 can not receive sensor clk! %x\n", mipi_reg);
                return -1;
            }
            msleep(10);
        }

        i = 0;
        /* wait for mipi stable */
        while (1) {
            mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
            if (!mipi_reg)
                break;
            if (i++ >= 20) {
                pr_err("mipi csi2 can not receive data correctly (x%X)\n",mipi_reg);
                return -1;
            }
            msleep(10);
        }

        pr_debug("receiving data");
        mipi_reg = mipi_csi2_get_error2(mipi_csi2_info);
        if (mipi_reg != 0)
        {
            pr_info("mipi_csi2 error2 = 0x%X\n", mipi_reg);
        }
        mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
        if ((mipi_reg != 0x300) && (mipi_reg != 0x330))
        {
            pr_info("mipi_csi2_dphy_status = 0x%X\n", mipi_reg);
        }

        ov5648_read_reg(0x0100, &resetval);
        if (!resetval&0x01) {
            pr_info("DEVICE WAS IN SOFTWARE STANDBY");
        }

        ov5648_write_reg(0x4800, 0x04);
        msleep(266);
    }
err:
    return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
    if (s == NULL) {
        pr_err("   ERROR!! no slave device set!\n");
        return -1;
    }

    memset(p, 0, sizeof(*p));
    p->u.bt656.clock_curr = ov5648_data.mclk;
    pr_debug("   clock_curr=mclk=%d\n", ov5648_data.mclk);
    p->if_type = V4L2_IF_TYPE_BT656;
    p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT; /* was V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT */
    p->u.bt656.clock_min = OV5648_XCLK_MIN;
    p->u.bt656.clock_max = OV5648_XCLK_MAX;
    p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync, was 1 */

    return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
    struct sensor_data *sensor = s->priv;
    pr_debug("ioctl_s_power(%d)\n", on);
    sensor->on = on;
    ov5648_standby(!on);

    return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
    struct sensor_data *sensor = s->priv;
    struct v4l2_captureparm *cparm = &a->parm.capture;
    int ret = 0;

    switch (a->type) {
    /* This is the only case currently handled. */
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        memset(a, 0, sizeof(*a));
        a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cparm->capability = sensor->streamcap.capability;
        cparm->timeperframe = sensor->streamcap.timeperframe;
        cparm->capturemode = sensor->streamcap.capturemode;
        ret = 0;
        break;

    /* These are all the possible cases. */
    case V4L2_BUF_TYPE_VIDEO_OUTPUT:
    case V4L2_BUF_TYPE_VIDEO_OVERLAY:
    case V4L2_BUF_TYPE_VBI_CAPTURE:
    case V4L2_BUF_TYPE_VBI_OUTPUT:
    case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
    case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
        ret = -EINVAL;
        break;

    default:
        pr_debug("   type is unknown - %d\n", a->type);
        ret = -EINVAL;
        break;
    }

    return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
    struct sensor_data *sensor = s->priv;
    struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
    u32 tgt_fps;    /* target frames per secound */
    enum ov5648_frame_rate frame_rate;
    enum ov5648_mode orig_mode;
    int ret = 0;

    switch (a->type) {
    /* This is the only case currently handled. */
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        /* Check that the new frame rate is allowed. */
        if ((timeperframe->numerator == 0) ||
            (timeperframe->denominator == 0)) {
            timeperframe->denominator = DEFAULT_FPS;
            timeperframe->numerator = 1;
        }

        tgt_fps = timeperframe->denominator /
              timeperframe->numerator;

        if (tgt_fps > MAX_FPS) {
            timeperframe->denominator = MAX_FPS;
            timeperframe->numerator = 1;
        } else if (tgt_fps < MIN_FPS) {
            timeperframe->denominator = MIN_FPS;
            timeperframe->numerator = 1;
        }

        /* Actual frame rate we use */
        tgt_fps = timeperframe->denominator /
              timeperframe->numerator;

        if (tgt_fps == 15)
            frame_rate = ov5648_15_fps;
        else if (tgt_fps == 30)
            frame_rate = ov5648_30_fps;
        else {
            pr_err(" The camera frame rate is not supported!\n");
            return -EINVAL;
        }

        orig_mode = sensor->streamcap.capturemode;
        ret = ov5648_init_mode(frame_rate,
                (u32)a->parm.capture.capturemode, orig_mode);
        if (ret < 0)
            return ret;

        sensor->streamcap.timeperframe = *timeperframe;
        sensor->streamcap.capturemode =
                (u32)a->parm.capture.capturemode;

        break;

    /* These are all the possible cases. */
    case V4L2_BUF_TYPE_VIDEO_OUTPUT:
    case V4L2_BUF_TYPE_VIDEO_OVERLAY:
    case V4L2_BUF_TYPE_VBI_CAPTURE:
    case V4L2_BUF_TYPE_VBI_OUTPUT:
    case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
    case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
        pr_debug("   type is not " \
            "V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
            a->type);
        ret = -EINVAL;
        break;

    default:
        pr_debug("   type is unknown - %d\n", a->type);
        ret = -EINVAL;
        break;
    }

    return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
    struct sensor_data *sensor = s->priv;

    pr_debug("In ov5648:ioctl_g_fmt_cap\n");

    switch (f->type) {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        f->fmt.pix = sensor->pix;
        pr_debug("%s: %dx%d\n", __func__, sensor->pix.width, sensor->pix.height);
        f->fmt.pix.pixelformat = ov5648_data.pix.pixelformat;
        break;

    case V4L2_BUF_TYPE_SENSOR:
        pr_debug("%s: left=%d, top=%d, %dx%d\n", __func__,
            sensor->spix.left, sensor->spix.top,
            sensor->spix.swidth, sensor->spix.sheight);
        f->fmt.spix = sensor->spix;
        f->fmt.pix.pixelformat = ov5648_data.pix.pixelformat;
        break;

    case V4L2_BUF_TYPE_PRIVATE:
        break;

    default:
        f->fmt.pix = sensor->pix;
        break;
    }

    return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
    int ret = 0;
    u8 reg;
    u8 reg_aux;

    pr_debug("In ov5648:ioctl_g_ctrl %d\n",
         vc->id);

    switch (vc->id) {
    case V4L2_CID_GLOWFORGE_SEL_CAM:
        vc->value = camera_num;
        break;
    case V4L2_CID_BRIGHTNESS:
        vc->value = ov5648_data.brightness;
        break;
    case V4L2_CID_HUE:
        vc->value = ov5648_data.hue;
        break;
    case V4L2_CID_CONTRAST:
        vc->value = ov5648_data.contrast;
        break;
    case V4L2_CID_SATURATION:
        vc->value = ov5648_data.saturation;
        break;
    case V4L2_CID_RED_BALANCE:
        vc->value = OV5648_get_color_gain(OV5648_RED);
        break;
    case V4L2_CID_BLUE_BALANCE:
        vc->value = OV5648_get_color_gain(OV5648_BLUE);
        break;
    case V4L2_CID_EXPOSURE_AUTO:
        ov5648_read_reg(0x3503, &reg);
        pr_debug("ov5648 AE control x%X\n",reg);
        vc->value = 0x01 & ~(reg & (0x01));
        break;
     case V4L2_CID_EXPOSURE: /* Exposure control */
        vc->value = OV5648_get_shutter();
        break;
    case V4L2_CID_AUTOGAIN: /* Automatic gain control */
        ov5648_read_reg(0x3503, &reg);
        vc->value = 0x01 & (~(reg & (0x02)) >> 1);
        break;
    case V4L2_CID_GAIN: /* Gain control */
        vc->value = OV5648_get_gain16();
        break;
    case V4L2_CID_AUTO_WHITE_BALANCE:
        ov5648_read_reg(0x5001, &reg);
        ov5648_read_reg(0x5002, &reg_aux);
        vc->value = ( (0x01 & reg_aux) << 1) | (reg & 0x01);
    case V4L2_CID_DO_WHITE_BALANCE:
        break;
    case V4L2_CID_FLASH_LED_MODE:
        ov5648_read_reg(0x3010,&reg);
        if (reg & 0x01)
        {
            vc->value = V4L2_FLASH_LED_MODE_TORCH;
        }
        else
        {
            vc->value = V4L2_FLASH_LED_MODE_NONE;
        }
        break;

    case V4L2_CID_FLASH_TORCH_INTENSITY:
        /* MODE_TORCH uses GPIO pin as torch signal */
        ov5648_read_reg(0x300D,&reg);
        vc->value = reg & 0x01;
        break;
    case V4L2_CID_HFLIP:
        ov5648_read_reg(0x3821,&reg);
        reg = reg & 0x06;
        if (reg == 0x06)
        {
            vc->value = 1;
            break;
        }
        if (reg == 0)
        {
            vc->value = 0;
            break;
        }
        vc->value = -1;
        ret = -EINVAL;
        break;
    case V4L2_CID_VFLIP:
        ov5648_read_reg(0x3820,&reg);
        reg = reg & 0x06;
        if (reg == 0x06)
        {
            vc->value = 1;
            break;
        }
        if (reg == 0)
        {
            vc->value = 0;
            break;
        }
        vc->value = -1;
        ret = -EINVAL;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
    int retval = 0;
    u8 reg;

    pr_debug("In ov5648:ioctl_s_ctrl %d\n",
         vc->id);

    switch (vc->id) {
    case V4L2_CID_BRIGHTNESS:
        break;
    case V4L2_CID_CONTRAST:
        break;
    case V4L2_CID_SATURATION:
        break;
    case V4L2_CID_HUE:
        break;
    case V4L2_CID_AUTO_WHITE_BALANCE:
        reg = vc->value & 0x01;
        ov5648_write_reg(0x5001, reg);

        reg = vc->value & 0x02;
        reg = reg >> 1;
        ov5648_write_reg(0x5002, reg);

        break;
    case V4L2_CID_DO_WHITE_BALANCE:
        if (vc->value == 1)
        {
            ov5648_read_reg(0x5180, &reg);
            reg = reg | 0x30;
            ov5648_write_reg(0x5180, reg);
            pr_debug("ov5648 DO WHITE c%X\n",reg);
        } else {
            ov5648_read_reg(0x5180, &reg);
            reg = reg & ~(0x30);
            ov5648_write_reg(0x5180, reg);
            pr_debug("ov5648 DO WHITE c%X\n",reg);
        }
        break;
    case V4L2_CID_RED_BALANCE:
        OV5648_set_color_gain(vc->value, OV5648_RED);
        break;
    case V4L2_CID_BLUE_BALANCE:
        OV5648_set_color_gain(vc->value, OV5648_BLUE);
        break;
    case V4L2_CID_GAMMA:
        break;
    case V4L2_CID_EXPOSURE_AUTO: /* Automatic exposure control */
        ov5648_read_reg(0x3503, &reg);
        if (0 == vc->value)
        {
            pr_debug("ov5648: Manual exposure\n");
            /* Use manual exposure control */
            reg = reg | 0x01;
        }
        else
        {
            pr_debug("ov5648: Auto exposure\n");
            /* Use auto exposure */
            reg = reg & ~(0x01);
        }
        ov5648_write_reg(0x3503, reg);
        break;
    case V4L2_CID_EXPOSURE: /* Exposure control */
        pr_debug("ov5648: Manual exposure value %d\n",vc->value);
        OV5648_set_shutter(vc->value);
        break;
    case V4L2_CID_AUTOGAIN: /* Automatic gain control */
        ov5648_read_reg(0x3503, &reg);
        if (0 == vc->value)
        {
            pr_debug("ov5648: Manual exposure\n");
            /* Use manual exposure control */
            reg = reg | 0x02;
        }
        else
        {
            pr_debug("ov5648: Auto exposure\n");
            /* Use auto exposure */
            reg = reg & ~(0x02);
        }
        ov5648_write_reg(0x3503, reg);
        break;
        break;
    case V4L2_CID_GAIN: /* Gain control */
        pr_debug("ov5648: Manual gain value %d\n",vc->value);
        OV5648_set_gain16(vc->value);
        break;
    case V4L2_CID_FLASH_LED_MODE:
        switch (vc->value)
        {
            case V4L2_FLASH_LED_MODE_NONE: /* Off. */
                /* MODE_TORCH uses GPIO pin as torch signal */
                /* Turn off */
                ov5648_read_reg(0x300D,&reg);
                reg = reg & ~(0x01);
                ov5648_write_reg(0x300D,reg);
                /* Pad out */
                ov5648_read_reg(0x3010,&reg);
                reg = reg & ~(0x01);
                ov5648_write_reg(0x3010,reg);
                break;
            case V4L2_FLASH_LED_MODE_TORCH: /* Torch mode. See V4L2_CID_FLASH_TORCH_INTENSITY. */
                /* MODE_TORCH uses GPIO pin as torch signal */
                /* Set output direction */
                ov5648_read_reg(0x3002,&reg);
                reg = reg | 0x01;
                ov5648_write_reg(0x3002,reg);
                /* Pad out */
                ov5648_read_reg(0x3010,&reg);
                reg = reg | 0x01;
                ov5648_write_reg(0x3010,reg);
                break;
            case V4L2_FLASH_LED_MODE_FLASH: /* Flash mode. */
                /* MODE_FLASH uses STROBE pin as strobe signal */
                /* Set output direction */
                ov5648_read_reg(0x3002,&reg);
                reg = reg | IO_STROBE_BIT;
                ov5648_write_reg(0x3002,reg);
                /* Pad out */
                ov5648_read_reg(SC_CMMN_PAD_SEL2,&reg);
                reg = reg | IO_STROBE_BIT;
                ov5648_write_reg(SC_CMMN_PAD_SEL2,reg);
                /* Configure the strobe mode */
                ov5648_read_reg(STROBE_RSTRB,&reg);
                /* LED 3 */
                reg = reg | 0x3;
                ov5648_write_reg(STROBE_RSTRB,reg);
                break;
        }
        break;
    case V4L2_CID_FLASH_STROBE_SOURCE:
        switch (vc->value)
        {
            case V4L2_FLASH_STROBE_SOURCE_SOFTWARE:
                strobe_software = 1;
            break;
            case V4L2_FLASH_STROBE_SOURCE_EXTERNAL:
                strobe_software = 0;
            break;
        }
        break;
    case V4L2_CID_FLASH_TORCH_INTENSITY:
        /* MODE_TORCH uses GPIO pin as torch signal */
        if (vc->value > 0)
        {
            ov5648_read_reg(0x300D,&reg);
            reg = reg | 0x01;
            ov5648_write_reg(0x300D,reg);
        }
        else
        {
            ov5648_read_reg(0x300D,&reg);
            reg = reg & ~(0x01);
            ov5648_write_reg(0x300D,reg);
        }
        break;
    case V4L2_CID_FLASH_STROBE:
        if (strobe_software)
        {

        }
    break;
    case V4L2_CID_FLASH_STROBE_STOP:
    break;
    case V4L2_CID_HFLIP:
        if (vc->value != 0)
        {
            ov5648_read_reg(0x3821,&reg);
            reg = reg | 0x06;
            ov5648_write_reg(0x3821,reg);
        }
        else
        {
            ov5648_read_reg(0x3821,&reg);
            reg = reg & ~(0x06);
            ov5648_write_reg(0x3821,reg);
        }
        break;
    case V4L2_CID_VFLIP:
        if (vc->value != 0)
        {
            ov5648_read_reg(0x3820,&reg);
            reg = reg | 0x06;
            ov5648_write_reg(0x3820,reg);
        }
        else
        {
            ov5648_read_reg(0x3820,&reg);
            reg = reg & ~(0x06);
            ov5648_write_reg(0x3820,reg);
        }
        break;
    case V4L2_CID_GLOWFORGE_SEL_CAM:
        if (vc->value == 0)
        {
            OV5648_stream_off();
            /* Change the camera */
            ov5648_data.i2c_client = i2c_client_master;
            camera_num = 0;
            if (gpio_is_valid(sel_gpio))
            {
                gpio_set_value(sel_gpio, !sel_active);
            }
#ifdef APPLY_TEST_PATTERN
    ov5648_write_reg(0x503D, 0x80);
#endif
            OV5648_stream_on();
        }
        else
        {
            if (NULL != i2c_client_slave)
            {
                OV5648_stream_off();
                /* Change the camera */
                ov5648_data.i2c_client = i2c_client_slave;
                camera_num = 1;
                if (gpio_is_valid(sel_gpio))
                {
                    gpio_set_value(sel_gpio, sel_active);
                }
#ifdef APPLY_TEST_PATTERN
    ov5648_write_reg(0x503D, 0x82);
#endif
                OV5648_stream_on();
            }
            else
            {
                pr_err("Error: invalid  i2c_client\n");
            }
        }

        break;
    default:
        retval = -EPERM;
        break;
    }

    return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *             VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
                 struct v4l2_frmsizeenum *fsize)
{
    if (fsize->index > ov5648_mode_MAX)
        return -EINVAL;

    fsize->pixel_format = ov5648_data.pix.pixelformat;
    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width =
            max(ov5648_mode_info_data[0][fsize->index].width,
                ov5648_mode_info_data[1][fsize->index].width);
    fsize->discrete.height =
            max(ov5648_mode_info_data[0][fsize->index].height,
                ov5648_mode_info_data[1][fsize->index].height);
    return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *          VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
    ((struct v4l2_dbg_chip_ident *)id)->match.type =
                    V4L2_CHIP_MATCH_I2C_DRIVER;
    strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
        "ov5648_mipi_camera");

    return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

    return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
                  struct v4l2_fmtdesc *fmt)
{
    if (fmt->index > ov5648_mode_MAX)
        return -EINVAL;

    fmt->pixelformat = ov5648_data.pix.pixelformat;

    return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
    struct sensor_data *sensor = s->priv;
    u32 tgt_xclk;   /* target xclk */
    u32 tgt_fps;    /* target frames per secound */
    int ret;
    enum ov5648_frame_rate frame_rate;
    void *mipi_csi2_info;

    ov5648_data.on = true;

    /* mclk */
    tgt_xclk = ov5648_data.mclk;
    tgt_xclk = min(tgt_xclk, (u32)OV5648_XCLK_MAX);
    tgt_xclk = max(tgt_xclk, (u32)OV5648_XCLK_MIN);
    ov5648_data.mclk = tgt_xclk;

    pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);

    /* Default camera frame rate is set in probe */
    tgt_fps = sensor->streamcap.timeperframe.denominator /
          sensor->streamcap.timeperframe.numerator;

    if (tgt_fps == 15)
        frame_rate = ov5648_15_fps;
    else if (tgt_fps == 30)
        frame_rate = ov5648_30_fps;
    else
        return -EINVAL; /* Only support 15fps or 30fps now. */

    mipi_csi2_info = mipi_csi2_get_info();

    /* enable mipi csi2 */
    if (mipi_csi2_info)
        mipi_csi2_enable(mipi_csi2_info);
    else {
        printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
               __func__, __FILE__);
        return -EPERM;
    }

    ret = ov5648_init_mode(frame_rate, ov5648_mode_INIT, ov5648_mode_INIT);

    return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
    void *mipi_csi2_info;

    mipi_csi2_info = mipi_csi2_get_info();

    /* disable mipi csi2 */
    if (mipi_csi2_info)
        if (mipi_csi2_get_status(mipi_csi2_info))
            mipi_csi2_disable(mipi_csi2_info);

    return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov5648_ioctl_desc[] = {
    {vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
    {vidioc_int_dev_exit_num, ioctl_dev_exit},
    {vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
    {vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
/*  {vidioc_int_g_needs_reset_num,
                (v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*  {vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
    {vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},
    {vidioc_int_enum_fmt_cap_num,
                (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*  {vidioc_int_try_fmt_cap_num,
                (v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
    {vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*  {vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
    {vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
    {vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*  {vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
    {vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
    {vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
    {vidioc_int_enum_framesizes_num,
                (v4l2_int_ioctl_func *) ioctl_enum_framesizes},
    {vidioc_int_g_chip_ident_num,
                (v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave ov5648_slave = {
    .ioctls = ov5648_ioctl_desc,
    .num_ioctls = ARRAY_SIZE(ov5648_ioctl_desc),
};

static struct v4l2_int_device ov5648_int_device = {
    .module = THIS_MODULE,
    .name = OV5648_NAME,
    .type = v4l2_int_type_slave,
    .u = {
        .slave = &ov5648_slave,
    },
};

static ssize_t show_reg(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    u8 val;
    s32 rval = ov5648_read_reg(ov5648_data.last_reg, &val);

    return sprintf(buf, "ov5648[0x%04x]=0x%02x\n",ov5648_data.last_reg, rval);
}
static ssize_t set_reg(struct device *dev,
            struct device_attribute *attr,
               const char *buf, size_t count)
{
    int regnum, value;
    int num_parsed = sscanf(buf, "%04x=%02x", &regnum, &value);
    if (1 <= num_parsed) {
        if (0xffff < (unsigned)regnum){
            pr_err("%s:invalid regnum %x\n", __func__, regnum);
            return 0;
        }
        ov5648_data.last_reg = regnum;
    }
    if (2 == num_parsed) {
        if (0xff < (unsigned)value) {
            pr_err("%s:invalid value %x\n", __func__, value);
            return 0;
        }
        ov5648_write_reg(ov5648_data.last_reg, value);
    }
    return count;
}

/* XXX: workaround for v4l2 client except for gstreamer-imx */
static ssize_t show_mode(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "ov5648 mode = 0x%02x\n", (int)ov5648_data.streamcap.capturemode);
}

static ssize_t set_mode(struct device *dev,
            struct device_attribute *attr,
               const char *buf, size_t count)
{
    unsigned long mode;

    mode = simple_strtoul(buf, NULL, 10);
    if ((enum ov5648_mode)mode >= ov5648_mode_MIN &&
        (enum ov5648_mode)mode <= ov5648_mode_MAX) {

        ov5648_data.streamcap.capturemode = mode;
        ov5648_data.pix.width =
            max(ov5648_mode_info_data[0][mode].width,
                ov5648_mode_info_data[1][mode].width);
        ov5648_data.pix.height =
            max(ov5648_mode_info_data[0][mode].height,
                ov5648_mode_info_data[1][mode].height);
        ov5648_data_add.map_sizeimage =
            ov5648_data.pix.width * ov5648_data.pix.height * 3 / 2;
    }

    return count;
}

static DEVICE_ATTR(ov5648_reg, S_IRUGO|S_IWUGO, show_reg, set_reg);
static DEVICE_ATTR(ov5648_mode, S_IRUGO|S_IWUGO, show_mode, set_mode);

void ov5648_register_slave_i2c(struct i2c_client * i2c)
{
    if (NULL != i2c)
    {

      i2c_client_slave = i2c;
      pr_info("i2c name %s\n",i2c_client_slave->name);
    }
    else
    {
      pr_err("ov5648_register_slave: invalid  i2c_client");
    }

}

EXPORT_SYMBOL_GPL(ov5648_register_slave_i2c);

/*!
 * ov5648 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5648_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    int retval, init;
    u8 chip_id_high, chip_id_low;
    struct sensor_data *sensor = &ov5648_data;
    enum of_gpio_flags flags;
    bool extbuf;

    /* request MIPI mux selection GPIO */
    sel_gpio = of_get_named_gpio_flags(dev->of_node, "sel-gpios", 0, &flags);
    if (gpio_is_valid(sel_gpio)) {
        /* sel_active - Select camera A */
        /* !sel_active - Select camera B */
        sel_active = !(flags & OF_GPIO_ACTIVE_LOW);
        init = (flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;

        retval = devm_gpio_request_one(dev, sel_gpio, init, "ov5648_mipi_sel");
        if (retval < 0) {
            dev_warn(dev, "request of sel_gpio failed");
            sel_gpio = -EINVAL;
        }
    }

    /* allocate extended video frame buffer */
    extbuf = of_find_property(dev->of_node, "extended-buffer", NULL);

    /* Set initial values for the sensor struct. */
    memset(&ov5648_data, 0, sizeof(ov5648_data));
    memset(&ov5648_data_add, 0, sizeof(ov5648_data_add));

    sensor->mipi_camera = 1;
    ov5648_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
    if (IS_ERR(ov5648_data.sensor_clk)) {
        /* assuming clock enabled by default */
        ov5648_data.sensor_clk = NULL;
        dev_err(dev, "clock-frequency missing or invalid\n");
        return PTR_ERR(ov5648_data.sensor_clk);
    }

    retval = of_property_read_u32(dev->of_node, "mclk",
                    &(ov5648_data.mclk));
    if (retval) {
        dev_err(dev, "mclk missing or invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "mclk_source",
                    (u32 *) &(ov5648_data.mclk_source));
    if (retval) {
        dev_err(dev, "mclk_source missing or invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "ipu_id",
                    &sensor->ipu_id);
    if (retval) {
        dev_err(dev, "ipu_id missing or invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "csi_id",
                    &(ov5648_data.csi));
    if (retval) {
        dev_err(dev, "csi id missing or invalid\n");
        return retval;
    }

    clk_prepare_enable(ov5648_data.sensor_clk);

    ov5648_data.io_init = ov5648_reset;
    ov5648_data.i2c_client = client;
    i2c_client_master = client;
    /* real OV5648 pixelformat is V4L2_PIX_FMT_SBGGR10.     */
    /* i.MX6 CSI CPD convert 10 bits color data to 8 bits.  */
    /* (see drivers/mxc/ipu3/ipu_capture.c - _ipu_csi_init) */
    ov5648_data.pix.pixelformat = V4L2_PIX_FMT_SBGGR8;
    ov5648_data.pix.width = 1024;
    ov5648_data.pix.height = 768;
    ov5648_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
                       V4L2_CAP_TIMEPERFRAME;
    ov5648_data.streamcap.capturemode = ov5648_mode_2592_1944;
    ov5648_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
    ov5648_data.streamcap.timeperframe.numerator = 1;

    /* lager memory allocate for Vivante direct texture mapping API */
    /* (VIDIOC_REQBUFS ioctl) */
    ov5648_data_add.map_sizeimage =
        ov5648_data.pix.width * ov5648_data.pix.height * 3 / 2; /* I420 */

    if (extbuf) {
        ov5648_data.adata = &ov5648_data_add;
    }

    ov5648_power_on(dev);

    ov5648_reset();

    ov5648_standby(0);

    retval = ov5648_read_reg(OV5648_CHIP_ID_HIGH_BYTE, &chip_id_high);
    if (retval < 0 || chip_id_high != 0x56) {
        pr_warning("camera ov5648_mipi is not found\n");
        clk_disable_unprepare(ov5648_data.sensor_clk);
        return -ENODEV;
    }
    retval = ov5648_read_reg(OV5648_CHIP_ID_LOW_BYTE, &chip_id_low);
    if (retval < 0 || chip_id_low != 0x48) {
        pr_warning("camera ov5648_mipi is not found\n");
        clk_disable_unprepare(ov5648_data.sensor_clk);
        return -ENODEV;
    }

    sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1);
    ov5648_standby(1);

    ov5648_int_device.priv = &ov5648_data;
    retval = v4l2_int_device_register(&ov5648_int_device);

//  clk_disable_unprepare(ov5648_data.sensor_clk);

    if (device_create_file(dev, &dev_attr_ov5648_reg))
        dev_err(dev, "%s: error creating ov5648_reg entry\n", __func__);
    if (device_create_file(dev, &dev_attr_ov5648_mode))
        dev_err(dev, "%s: error creating ov5648_mode entry\n", __func__);

    pr_info("camera ov5648_mipi is found\n");
    return retval;
}

/*!
 * ov5648 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5648_remove(struct i2c_client *client)
{
    v4l2_int_device_unregister(&ov5648_int_device);

    return 0;
}

/*!
 * ov5648 init function
 * Called by insmod ov5648_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov5648_init(void)
{
    u8 err;
    pr_info("Loading ov5648_mipi driver\n");
    err = i2c_add_driver(&ov5648_i2c_driver);
    if (err != 0)
        pr_err("%s:driver registration failed, error=%d\n",
            __func__, err);
    pr_info("Loaded ov5648_mipi driver\n");
    return err;
}

/*!
 * OV5648 cleanup function
 * Called on rmmod ov5648_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov5648_clean(void)
{
    i2c_del_driver(&ov5648_i2c_driver);
}

module_init(ov5648_init);
module_exit(ov5648_clean);

MODULE_AUTHOR("Glowforge, Inc. <opensource@glowforge.com>");
MODULE_DESCRIPTION("OV5648 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

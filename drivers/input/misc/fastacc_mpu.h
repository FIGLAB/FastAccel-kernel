/*
 * fastacc_mpu.c - High-speed accelerometer driver for InvenSense MPU devices
 *
 * Copyright (C) 2017 Robert Xiao <brx@cs.cmu.edu>
 *     Future Interfaces Group, Carnegie Mellon University
 *
 * If you use this code, we ask you cite the originating paper, ViBand:
 *
 *   Gierad Laput, Robert Xiao, and Chris Harrison. 2016. ViBand: High-Fidelity
 *   Bio-Acoustic Sensing Using Commodity Smartwatch Accelerometers.
 *   In Proceedings of the 29th Annual Symposium on User Interface Software and
 *   Technology (UIST '16). ACM, New York, NY, USA, 321-333.
 *   DOI: https://doi.org/10.1145/2984511.2984582
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

#ifndef _FASTACC_MPU_H
#define _FASTACC_MPU_H

/* Register information derived from merging the MPU-6555 and MPU-9250 register maps */
enum mpu_reg {
    MPU_SELF_TEST_X_GYRO = 0x00,
    MPU_SELF_TEST_Y_GYRO = 0x01,
    MPU_SELF_TEST_Z_GYRO = 0x02,
    MPU_SELF_TEST_X_ACCEL = 0x0D,
    MPU_SELF_TEST_Y_ACCEL = 0x0E,
    MPU_SELF_TEST_Z_ACCEL = 0x0F,
    MPU_XG_OFFSET_H = 0x13,
    MPU_XG_OFFSET_L = 0x14,
    MPU_YG_OFFSET_H = 0x15,
    MPU_YG_OFFSET_L = 0x16,
    MPU_ZG_OFFSET_H = 0x17,
    MPU_ZG_OFFSET_L = 0x18,
    MPU_SMPLRT_DIV = 0x19,
    MPU_CONFIG = 0x1A,
    MPU_GYRO_CONFIG = 0x1B,
    MPU_ACCEL_CONFIG = 0x1C,
    MPU_ACCEL_CONFIG_2 = 0x1D,
    MPU_LP_ACCEL_ODR = 0x1E,
    MPU_WOM_THR = 0x1F,
    MPU_FIFO_EN = 0x23,
    MPU_I2C_MST_CTRL = 0x24,
    MPU_I2C_SLV0_ADDR = 0x25,
    MPU_I2C_SLV0_REG = 0x26,
    MPU_I2C_SLV0_CTRL = 0x27,
    MPU_I2C_SLV1_ADDR = 0x28,
    MPU_I2C_SLV1_REG = 0x29,
    MPU_I2C_SLV1_CTRL = 0x2A,
    MPU_I2C_SLV2_ADDR = 0x2B,
    MPU_I2C_SLV2_REG = 0x2C,
    MPU_I2C_SLV2_CTRL = 0x2D,
    MPU_I2C_SLV3_ADDR = 0x2E,
    MPU_I2C_SLV3_REG = 0x2F,
    MPU_I2C_SLV3_CTRL = 0x30,
    MPU_I2C_SLV4_ADDR = 0x31,
    MPU_I2C_SLV4_REG = 0x32,
    MPU_I2C_SLV4_DO = 0x33,
    MPU_I2C_SLV4_CTRL = 0x34,
    MPU_I2C_SLV4_DI = 0x35,
    MPU_I2C_MST_STATUS = 0x36,
    MPU_INT_PIN_CFG = 0x37,
    MPU_INT_ENABLE = 0x38,
    MPU_INT_STATUS = 0x3A,
    MPU_ACCEL_XOUT_H = 0x3B,
    MPU_ACCEL_XOUT_L = 0x3C,
    MPU_ACCEL_YOUT_H = 0x3D,
    MPU_ACCEL_YOUT_L = 0x3E,
    MPU_ACCEL_ZOUT_H = 0x3F,
    MPU_ACCEL_ZOUT_L = 0x40,
    MPU_TEMP_OUT_H = 0x41,
    MPU_TEMP_OUT_L = 0x42,
    MPU_GYRO_XOUT_H = 0x43,
    MPU_GYRO_XOUT_L = 0x44,
    MPU_GYRO_YOUT_H = 0x45,
    MPU_GYRO_YOUT_L = 0x46,
    MPU_GYRO_ZOUT_H = 0x47,
    MPU_GYRO_ZOUT_L = 0x48,
    MPU_EXT_SENS_DATA_00 = 0x49,
    MPU_EXT_SENS_DATA_01 = 0x4A,
    MPU_EXT_SENS_DATA_02 = 0x4B,
    MPU_EXT_SENS_DATA_03 = 0x4C,
    MPU_EXT_SENS_DATA_04 = 0x4D,
    MPU_EXT_SENS_DATA_05 = 0x4E,
    MPU_EXT_SENS_DATA_06 = 0x4F,
    MPU_EXT_SENS_DATA_07 = 0x50,
    MPU_EXT_SENS_DATA_08 = 0x51,
    MPU_EXT_SENS_DATA_09 = 0x52,
    MPU_EXT_SENS_DATA_10 = 0x53,
    MPU_EXT_SENS_DATA_11 = 0x54,
    MPU_EXT_SENS_DATA_12 = 0x55,
    MPU_EXT_SENS_DATA_13 = 0x56,
    MPU_EXT_SENS_DATA_14 = 0x57,
    MPU_EXT_SENS_DATA_15 = 0x58,
    MPU_EXT_SENS_DATA_16 = 0x59,
    MPU_EXT_SENS_DATA_17 = 0x5A,
    MPU_EXT_SENS_DATA_18 = 0x5B,
    MPU_EXT_SENS_DATA_19 = 0x5C,
    MPU_EXT_SENS_DATA_20 = 0x5D,
    MPU_EXT_SENS_DATA_21 = 0x5E,
    MPU_EXT_SENS_DATA_22 = 0x5F,
    MPU_EXT_SENS_DATA_23 = 0x60,
    MPU_I2C_SLV0_DO = 0x63,
    MPU_I2C_SLV1_DO = 0x64,
    MPU_I2C_SLV2_DO = 0x65,
    MPU_I2C_SLV3_DO = 0x66,
    MPU_I2C_MST_DELAY_CTRL = 0x67,
    MPU_SIGNAL_PATH_RESET = 0x68,
    MPU_ACCEL_INTEL_CTRL = 0x69,
    MPU_USER_CTRL = 0x6A,
    MPU_PWR_MGMT_1 = 0x6B,
    MPU_PWR_MGMT_2 = 0x6C,
    MPU_FIFO_COUNTH = 0x72,
    MPU_FIFO_COUNTL = 0x73,
    MPU_FIFO_R_W = 0x74,
    MPU_WHO_AM_I = 0x75,
    MPU_XA_OFFSET_H = 0x77,
    MPU_XA_OFFSET_L = 0x78,
    MPU_YA_OFFSET_H = 0x7A,
    MPU_YA_OFFSET_L = 0x7B,
    MPU_ZA_OFFSET_H = 0x7D,
    MPU_ZA_OFFSET_L = 0x7E,
};

/* Flags */
#define MPU_CONFIG_FIFO_MODE 0x40
#define MPU_CONFIG_EXT_SYNC_SET_NONE (0<<3)
#define MPU_CONFIG_EXT_SYNC_SET_TEMP (1<<3)
#define MPU_CONFIG_EXT_SYNC_SET_GYROX (2<<3)
#define MPU_CONFIG_EXT_SYNC_SET_GYROY (3<<3)
#define MPU_CONFIG_EXT_SYNC_SET_GYROZ (4<<3)
#define MPU_CONFIG_EXT_SYNC_SET_ACCELX (5<<3)
#define MPU_CONFIG_EXT_SYNC_SET_ACCELY (6<<3)
#define MPU_CONFIG_EXT_SYNC_SET_ACCELZ (7<<3)
#define MPU_CONFIG_EXT_SYNC_SET_MASK (7<<3)
/* Low pass filter for Gyro and Temp sensor */
#define MPU_CONFIG_DLPF_CFG_0 0
#define MPU_CONFIG_DLPF_CFG_1 1
#define MPU_CONFIG_DLPF_CFG_2 2
#define MPU_CONFIG_DLPF_CFG_3 3
#define MPU_CONFIG_DLPF_CFG_4 4
#define MPU_CONFIG_DLPF_CFG_5 5
#define MPU_CONFIG_DLPF_CFG_6 6
#define MPU_CONFIG_DLPF_CFG_7 7
#define MPU_CONFIG_DLPF_CFG_MASK 7

#define MPU_GYRO_CONFIG_XGYRO_SELFTEST 0x80
#define MPU_GYRO_CONFIG_YGYRO_SELFTEST 0x40
#define MPU_GYRO_CONFIG_ZGYRO_SELFTEST 0x20
#define MPU_GYRO_CONFIG_GYRO_FS_SEL_250_DPS (0<<3)
#define MPU_GYRO_CONFIG_GYRO_FS_SEL_500_DPS (1<<3)
#define MPU_GYRO_CONFIG_GYRO_FS_SEL_1000_DPS (2<<3)
#define MPU_GYRO_CONFIG_GYRO_FS_SEL_2000_DPS (3<<3)
#define MPU_GYRO_CONFIG_GYRO_FS_SEL_MASK (3<<3)
/* GYRO_CONFIG.Fchoice_b is inverted version of Fchoice */
#define MPU_GYRO_CONFIG_GYRO_FCHOICE_00 3
#define MPU_GYRO_CONFIG_GYRO_FCHOICE_01 2
#define MPU_GYRO_CONFIG_GYRO_FCHOICE_10 1
#define MPU_GYRO_CONFIG_GYRO_FCHOICE_11 0
#define MPU_GYRO_CONFIG_GYRO_FCHOICE_MASK 3

#define MPU_ACCEL_CONFIG_XACCEL_SELFTEST 0x80
#define MPU_ACCEL_CONFIG_YACCEL_SELFTEST 0x40
#define MPU_ACCEL_CONFIG_ZACCEL_SELFTEST 0x20
#define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_2_G (0<<3)
#define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_4_G (1<<3)
#define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_8_G (2<<3)
#define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_16_G (3<<3)
#define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_MASK (3<<3)

#define MPU_ACCEL_CONFIG_2_FIFO_SIZE_512_B (0<<6) /* default */
#define MPU_ACCEL_CONFIG_2_FIFO_SIZE_1024_B (1<<6)
#define MPU_ACCEL_CONFIG_2_FIFO_SIZE_2048_B (2<<6)
#define MPU_ACCEL_CONFIG_2_FIFO_SIZE_4096_B (3<<6)
#define MPU_ACCEL_CONFIG_2_FIFO_SIZE_MASK (3<<6)
/* ACCEL_CONFIG_2.ACCEL_FCHOICE_B is inverted version of ACCEL_FCHOICE */
#define MPU_ACCEL_CONFIG_2_ACCEL_FCHOICE_0 (1<<3)
#define MPU_ACCEL_CONFIG_2_ACCEL_FCHOICE_1 (0<<3)
#define MPU_ACCEL_CONFIG_2_ACCEL_FCHOICE_MASK 1
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_0 0
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_1 1
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_2 2
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_3 3
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_4 4
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_5 5
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_6 6
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_7 7
#define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_MASK 7

#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_0_24_HZ 0
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_0_49_HZ 1
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_0_98_HZ 2
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_1_95_HZ 3
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_3_91_HZ 4
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_7_81_HZ 5
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_15_63_HZ 6
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_31_25_HZ 7
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_62_50_HZ 8
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_125_HZ 9
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_250_HZ 10
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_500_HZ 11
#define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_MASK 15

#define MPU_FIFO_EN_TEMP 0x80
#define MPU_FIFO_EN_GYROX 0x40
#define MPU_FIFO_EN_GYROY 0x20
#define MPU_FIFO_EN_GYROZ 0x10
#define MPU_FIFO_EN_ACCEL 0x8
#define MPU_FIFO_EN_SLV_2 0x4
#define MPU_FIFO_EN_SLV_1 0x2
#define MPU_FIFO_EN_SLV_0 0x1

#define MPU_I2C_MST_CTRL_MULT_MST_EN 0x80
#define MPU_I2C_MST_CTRL_WAIT_FOR_ES 0x40
#define MPU_I2C_MST_CTRL_SLV_3_FIFO_EN 0x20
#define MPU_I2C_MST_CTRL_I2C_MST_P_NSR 0x10
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_348_KHZ 0
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_333_KHZ 1
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_320_KHZ 2
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_308_KHZ 3
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_296_KHZ 4
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_286_KHZ 5
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_276_KHZ 6
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_267_KHZ 7
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_258_KHZ 8
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_500_KHZ 9
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_471_KHZ 10
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_444_KHZ 11
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_421_KHZ 12
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_400_KHZ 13
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_381_KHZ 14
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_364_KHZ 15
#define MPU_I2C_MST_CTRL_I2C_MST_CLK_MASK 15

/* TODO: rest of the I2C flags */

#define MPU_INT_PIN_CFG_ACTL 0x80
#define MPU_INT_PIN_CFG_OPEN 0x40
#define MPU_INT_PIN_CFG_LATCH_INT_EN 0x20
#define MPU_INT_PIN_CFG_INT_ANYRD_2CLEAR 0x10
#define MPU_INT_PIN_CFG_ACTL_FSYNC 0x8
#define MPU_INT_PIN_CFG_FSYNC_INT_MODE_EN 0x4
#define MPU_INT_PIN_CFG_BYPASS_EN 0x2

#define MPU_INT_ENABLE_WOM 0x40
#define MPU_INT_ENABLE_FIFO_OVERFLOW 0x10
#define MPU_INT_ENABLE_FSYNC_INT 0x8
#define MPU_INT_ENABLE_RAW_RDY 0x1

#define MPU_INT_STATUS_WOM 0x40
#define MPU_INT_STATUS_FIFO_OVERFLOW 0x10
#define MPU_INT_STATUS_FSYNC_INT 0x8
#define MPU_INT_STATUS_DMP 0x2
#define MPU_INT_STATUS_RAW_RDY 0x1

#define MPU_SIGNAL_PATH_RESET_GYRO 0x4
#define MPU_SIGNAL_PATH_RESET_ACCEL 0x2
#define MPU_SIGNAL_PATH_RESET_TEMP 0x1

#define MPU_ACCEL_INTEL_CTRL_ENABLE 0x80
#define MPU_ACCEL_INTEL_CTRL_MODE 0x40

#define MPU_USER_CTRL_DMP_EN 0x80
#define MPU_USER_CTRL_FIFO_EN 0x40
#define MPU_USER_CTRL_I2C_MST_EN 0x20
#define MPU_USER_CTRL_I2C_IF_DIS 0x10
#define MPU_USER_CTRL_DMP_RST 0x8
#define MPU_USER_CTRL_FIFO_RST 0x4
#define MPU_USER_CTRL_I2C_MST_RST 0x2
#define MPU_USER_CTRL_SIG_COND_RST 0x1

#define MPU_PWR_MGMT_1_DEVICE_RESET 0x80
#define MPU_PWR_MGMT_1_SLEEP 0x40
#define MPU_PWR_MGMT_1_CYCLE 0x20
#define MPU_PWR_MGMT_1_GYRO_STANDBY 0x10
#define MPU_PWR_MGMT_1_TEMP_DIS 0x8
#define MPU_PWR_MGMT_1_CLKSEL_INTERNAL 0 /* internal 20 MHz osc */
#define MPU_PWR_MGMT_1_CLKSEL_AUTO 1 /* internal 20 MHz osc */
/* 1-5 are all "auto" */
/* 6 is internal again */
#define MPU_PWR_MGMT_1_CLKSEL_STOP 7
#define MPU_PWR_MGMT_1_CLKSEL_MASK 7

#define MPU_PWR_MGMT_2_LP_WAKE_CTRL_1_25_HZ (0<<6)
#define MPU_PWR_MGMT_2_LP_WAKE_CTRL_5_HZ (1<<6)
#define MPU_PWR_MGMT_2_LP_WAKE_CTRL_20_HZ (2<<6)
#define MPU_PWR_MGMT_2_LP_WAKE_CTRL_40_HZ (3<<6)
#define MPU_PWR_MGMT_2_LP_WAKE_CTRL_MASK (3<<6)
#define MPU_PWR_MGMT_2_DISABLE_XA 0x20
#define MPU_PWR_MGMT_2_DISABLE_YA 0x10
#define MPU_PWR_MGMT_2_DISABLE_ZA 0x8
#define MPU_PWR_MGMT_2_DISABLE_XG 0x4
#define MPU_PWR_MGMT_2_DISABLE_YG 0x2
#define MPU_PWR_MGMT_2_DISABLE_ZG 0x1

/* A brief, incomplete list of WHO_AM_I responses */
#define MPU_WHO_AM_I_MPU_6050_9150 0x68
#define MPU_WHO_AM_I_MPU_6555 0x7C
#define MPU_WHO_AM_I_MPU_6500_9250 0x70
#define MPU_WHO_AM_I_MPU_9250 0x71
#define MPU_WHO_AM_I_MPU_9255 0x73
/* n.b. LG G Watch MPU responds 0x74 - teardown says it's a 6515? */

#endif /* _FASTACC_MPU_H */

/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #![allow(non_camel_case_types)]

pub const BMI270_REG_CHIP_ID  		:u8= 0x00;
pub const BMI270_REG_ERROR    		:u8= 0x02;
pub const BMI270_REG_STATUS			:u8= 0x03;
pub const BMI270_REG_AUX_X_LSB  	:u8= 0x04;
pub const BMI270_REG_ACC_X_LSB  	:u8= 0x0C;
pub const BMI270_REG_GYR_X_LSB  	:u8= 0x12;
pub const BMI270_REG_SENSORTIME_0 	:u8= 0x18;
pub const BMI270_REG_EVENT        	:u8= 0x1B;
pub const BMI270_REG_INT_STATUS_0  	:u8= 0x1C;
pub const BMI270_REG_SC_OUT_0      	:u8= 0x1E;
pub const BMI270_REG_WR_GEST_ACT   	:u8= 0x20;
pub const BMI270_REG_INTERNAL_STATUS :u8=0x21;
pub const BMI270_REG_TEMPERATURE_0   :u8=0x22;
pub const BMI270_REG_FIFO_LENGTH_0   :u8=0x24;
pub const BMI270_REG_FIFO_DATA     :u8=  0x26;
pub const BMI270_REG_FEAT_PAGE     :u8=  0x2F;
pub const BMI270_REG_FEATURES_0    :u8=  0x30;
pub const BMI270_REG_ACC_CONF      :u8=  0x40;
pub const BMI270_REG_ACC_RANGE     :u8=  0x41;
pub const BMI270_REG_GYR_CONF      :u8=  0x42;
pub const BMI270_REG_GYR_RANGE     :u8=  0x43;
pub const BMI270_REG_AUX_CONF      :u8=  0x44;
pub const BMI270_REG_FIFO_DOWNS    :u8=  0x45;
pub const BMI270_REG_FIFO_WTM_0    :u8=  0x46;
pub const BMI270_REG_FIFO_CONFIG_0 :u8=  0x48;
pub const BMI270_REG_SATURATION    :u8=  0x4A;
pub const BMI270_REG_AUX_DEV_ID    :u8=  0x4B;
pub const BMI270_REG_AUX_IF_CONF   :u8=  0x4C;
pub const BMI270_REG_AUX_RD_ADDR   :u8=  0x4D;
pub const BMI270_REG_AUX_WR_ADDR   :u8=  0x4E;
pub const BMI270_REG_AUX_WR_DATA   :u8=  0x4F;
pub const BMI270_REG_ERR_REG_MSK   :u8=  0x52;
pub const BMI270_REG_INT1_IO_CTRL  :u8=  0x53;
pub const BMI270_REG_INT2_IO_CTRL  :u8=  0x54;
pub const BMI270_REG_INT_LATCH     :u8=  0x55;
pub const BMI270_REG_INT1_MAP_FEAT :u8=  0x56;
pub const BMI270_REG_INT2_MAP_FEAT :u8=  0x57;
pub const BMI270_REG_INT_MAP_DATA  :u8=  0x58;
pub const BMI270_REG_INIT_CTRL     :u8=  0x59;
pub const BMI270_REG_INIT_ADDR_0   :u8=  0x5B;
pub const BMI270_REG_INIT_DATA     :u8=  0x5E;
pub const BMI270_REG_INTERNAL_ERROR :u8= 0x5F;
pub const BMI270_REG_AUX_IF_TRIM    :u8= 0x68;
pub const BMI270_REG_GYR_CRT_CONF   :u8= 0x69;
pub const BMI270_REG_NVM_CONF       :u8= 0x6A;
pub const BMI270_REG_IF_CONF        :u8= 0x6B;
pub const BMI270_REG_DRV            :u8= 0x6C;
pub const BMI270_REG_ACC_SELF_TEST  :u8= 0x6D;
pub const BMI270_REG_GYR_SELF_TEST  :u8= 0x6E;
pub const BMI270_REG_NV_CONF        :u8= 0x70;
pub const BMI270_REG_OFFSET_0       :u8= 0x71;
pub const BMI270_REG_PWR_CONF       :u8= 0x7C;
pub const BMI270_REG_PWR_CTRL       :u8= 0x7D;
pub const BMI270_REG_CMD            :u8= 0x7E;
/* TODO
pub const BMI270_REG_MASK            GENMASK(6, 0)

pub const BMI270_ANYMO_1_DURATION_POS	0
pub const BMI270_ANYMO_1_DURATION_MASK	BIT_MASK(12)
pub const BMI270_ANYMO_1_DURATION(n)	((n) << BMI270_ANYMO_1_DURATION_POS)
pub const BMI270_ANYMO_1_SELECT_X		BIT(13)
pub const BMI270_ANYMO_1_SELECT_Y		BIT(14)
pub const BMI270_ANYMO_1_SELECT_Z		BIT(15)
pub const BMI270_ANYMO_1_SELECT_XYZ	(BMI270_ANYMO_1_SELECT_X | \
					 BMI270_ANYMO_1_SELECT_Y | \
					 BMI270_ANYMO_1_SELECT_Y)
pub const BMI270_ANYMO_2_THRESHOLD_POS	0
pub const BMI270_ANYMO_2_THRESHOLD_MASK	BIT_MASK(10)
pub const BMI270_ANYMO_2_THRESHOLD(n)	((n) << BMI270_ANYMO_2_THRESHOLD_POS)
pub const BMI270_ANYMO_2_OUT_CONF_POS	11
pub const BMI270_ANYMO_2_OUT_CONF_MASK	(BIT(11) | BIT(12) | BIT(13) | BIT(14))
pub const BMI270_ANYMO_2_ENABLE		BIT(15)
pub const BMI270_ANYMO_2_OUT_CONF_OFF	(0x00 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_0	(0x01 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_1	(0x02 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_2	(0x03 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_3	(0x04 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_4	(0x05 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_5	(0x06 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_6	(0x07 << BMI270_ANYMO_2_OUT_CONF_POS)
pub const BMI270_ANYMO_2_OUT_CONF_BIT_8	(0x08 << BMI270_ANYMO_2_OUT_CONF_POS)

pub const BMI270_INT_IO_CTRL_LVL		BIT(1) /* Output level (0 = active low, 1 = active high) */
pub const BMI270_INT_IO_CTRL_OD		BIT(2) /* Open-drain (0 = push-pull, 1 = open-drain)*/
pub const BMI270_INT_IO_CTRL_OUTPUT_EN	BIT(3) /* Output enabled */
pub const BMI270_INT_IO_CTRL_INPUT_EN	BIT(4) /* Input enabled */

/* Applies to INT1_MAP_FEAT, INT2_MAP_FEAT, INT_STATUS_0 */
pub const BMI270_INT_MAP_SIG_MOTION        BIT(0)
pub const BMI270_INT_MAP_STEP_COUNTER      BIT(1)
pub const BMI270_INT_MAP_ACTIVITY          BIT(2)
pub const BMI270_INT_MAP_WRIST_WEAR_WAKEUP BIT(3)
pub const BMI270_INT_MAP_WRIST_GESTURE     BIT(4)
pub const BMI270_INT_MAP_NO_MOTION         BIT(5)
pub const BMI270_INT_MAP_ANY_MOTION        BIT(6)

pub const BMI270_INT_MAP_DATA_FFULL_INT1		BIT(0)
pub const BMI270_INT_MAP_DATA_FWM_INT1		BIT(1)
pub const BMI270_INT_MAP_DATA_DRDY_INT1		BIT(2)
pub const BMI270_INT_MAP_DATA_ERR_INT1		BIT(3)
pub const BMI270_INT_MAP_DATA_FFULL_INT2		BIT(4)
pub const BMI270_INT_MAP_DATA_FWM_INT2		BIT(5)
pub const BMI270_INT_MAP_DATA_DRDY_INT2		BIT(6)
pub const BMI270_INT_MAP_DATA_ERR_INT2		BIT(7)

pub const BMI270_INT_STATUS_ANY_MOTION		BIT(6)
*/
pub const BMI270_CHIP_ID :u8 = 0x24;

pub const BMI270_CMD_G_TRIGGER :u8 =  0x02;
pub const BMI270_CMD_USR_GAIN  :u8 =  0x03;
pub const BMI270_CMD_NVM_PROG  :u8 =  0xA0;
pub const BMI270_CMD_FIFO_FLUSH :u8 = 0xB0;
pub const BMI270_CMD_SOFT_RESET :u8 = 0xB6;

pub const BMI270_POWER_ON_TIME                :u32 = 800;
pub const BMI270_SOFT_RESET_TIME           	  :u32 = 2000;
pub const BMI270_ACC_SUS_TO_NOR_START_UP_TIME :u32 = 2000;
pub const BMI270_GYR_SUS_TO_NOR_START_UP_TIME :u32 = 45000;
pub const BMI270_GYR_FAST_START_UP_TIME       :u32 = 2000;
pub const BMI270_TRANSC_DELAY_SUSPEND         :u32 = 450;
pub const BMI270_TRANSC_DELAY_NORMAL          :u32 = 2;

pub const BMI270_PREPARE_CONFIG_LOAD  :u8 = 0x00;
pub const BMI270_COMPLETE_CONFIG_LOAD :u8 = 0x01;

pub const BMI270_INST_MESSAGE_MSK        :u8 = 0x0F;
pub const BMI270_INST_MESSAGE_NOT_INIT   :u8 = 0x00;
pub const BMI270_INST_MESSAGE_INIT_OK    :u8 = 0x01;
pub const BMI270_INST_MESSAGE_INIT_ERR   :u8 = 0x02;
pub const BMI270_INST_MESSAGE_DRV_ERR    :u8 = 0x03;
pub const BMI270_INST_MESSAGE_SNS_STOP   :u8 = 0x04;
pub const BMI270_INST_MESSAGE_NVM_ERR    :u8 = 0x05;
pub const BMI270_INST_MESSAGE_STRTUP_ERR :u8 = 0x06;
pub const BMI270_INST_MESSAGE_COMPAT_ERR :u8 = 0x07;

pub const BMI270_INST_AXES_REMAP_ERROR :u8 = 0x20;
pub const BMI270_INST_ODR_50HZ_ERROR   :u8 = 0x40;

pub const BMI270_PWR_CONF_ADV_PWR_SAVE_MSK :u8 = 0x01;
pub const BMI270_PWR_CONF_ADV_PWR_SAVE_EN  :u8 = 0x01;
pub const BMI270_PWR_CONF_ADV_PWR_SAVE_DIS :u8 = 0x00;

pub const BMI270_PWR_CONF_FIFO_SELF_WKUP_MSK :u8 = 0x02;
pub const BMI270_PWR_CONF_FIFO_SELF_WKUP_POS :u8 = 0x01;
pub const BMI270_PWR_CONF_FIFO_SELF_WKUP_EN  :u8 = 0x01;
pub const BMI270_PWR_CONF_FIFO_SELF_WKUP_DIS :u8 = 0x00;

pub const BMI270_PWR_CONF_FUP_EN_MSK :u8 = 0x04;
pub const BMI270_PWR_CONF_FUP_EN_POS :u8 = 0x02;
pub const BMI270_PWR_CONF_FUP_EN     :u8 = 0x01;
pub const BMI270_PWR_CONF_FUP_DIS    :u8 = 0x00;

pub const BMI270_PWR_CTRL_MSK     :u8 = 0x0F;
pub const BMI270_PWR_CTRL_AUX_EN  :u8 = 0x01;
pub const BMI270_PWR_CTRL_GYR_EN  :u8 = 0x02;
pub const BMI270_PWR_CTRL_ACC_EN  :u8 = 0x04;
pub const BMI270_PWR_CTRL_TEMP_EN :u8 = 0x08;

pub const BMI270_ACC_ODR_MSK      :u8 = 0x0F;
pub const BMI270_ACC_ODR_25D32_HZ :u8 = 0x01;
pub const BMI270_ACC_ODR_25D16_HZ :u8 = 0x02;
pub const BMI270_ACC_ODR_25D8_HZ  :u8 = 0x03;
pub const BMI270_ACC_ODR_25D4_HZ  :u8 = 0x04;
pub const BMI270_ACC_ODR_25D2_HZ  :u8 = 0x05;
pub const BMI270_ACC_ODR_25_HZ    :u8 = 0x06;
pub const BMI270_ACC_ODR_50_HZ    :u8 = 0x07;
pub const BMI270_ACC_ODR_100_HZ   :u8 = 0x08;
pub const BMI270_ACC_ODR_200_HZ   :u8 = 0x09;
pub const BMI270_ACC_ODR_400_HZ   :u8 = 0x0A;
pub const BMI270_ACC_ODR_800_HZ   :u8 = 0x0B;
pub const BMI270_ACC_ODR_1600_HZ  :u8 = 0x0C;

pub const BMI270_ACC_BWP_MSK        :u8 = 0x30;
pub const BMI270_ACC_BWP_POS        :u8 = 4;
pub const BMI270_ACC_BWP_OSR4_AVG1  :u8 = 0x00;
pub const BMI270_ACC_BWP_OSR2_AVG2  :u8 = 0x01;
pub const BMI270_ACC_BWP_NORM_AVG4  :u8 = 0x02;
pub const BMI270_ACC_BWP_CIC_AVG8   :u8 = 0x03;
pub const BMI270_ACC_BWP_RES_AVG16  :u8 = 0x04;
pub const BMI270_ACC_BWP_RES_AVG32  :u8 = 0x05;
pub const BMI270_ACC_BWP_RES_AVG64  :u8 = 0x06;
pub const BMI270_ACC_BWP_RES_AVG128 :u8 = 0x07;

pub const BMI270_ACC_FILT_MSK      :u8 = 0x80;
pub const BMI270_ACC_FILT_POS      :u8 = 7;
pub const BMI270_ACC_FILT_PWR_OPT  :u8 = 0x00;
pub const BMI270_ACC_FILT_PERF_OPT :u8 = 0x01;

pub const BMI270_ACC_RANGE_MSK :u8 = 0x03;
pub const BMI270_ACC_RANGE_2G  :u8 = 0x00;
pub const BMI270_ACC_RANGE_4G  :u8 = 0x01;
pub const BMI270_ACC_RANGE_8G  :u8 = 0x02;
pub const BMI270_ACC_RANGE_16G :u8 = 0x03;

pub const BMI270_GYR_ODR_MSK     :u8 = 0x0F;
pub const BMI270_GYR_ODR_25_HZ   :u8 = 0x06;
pub const BMI270_GYR_ODR_50_HZ   :u8 = 0x07;
pub const BMI270_GYR_ODR_100_HZ  :u8 = 0x08;
pub const BMI270_GYR_ODR_200_HZ  :u8 = 0x09;
pub const BMI270_GYR_ODR_400_HZ  :u8 = 0x0A;
pub const BMI270_GYR_ODR_800_HZ  :u8 = 0x0B;
pub const BMI270_GYR_ODR_1600_HZ :u8 = 0x0C;
pub const BMI270_GYR_ODR_3200_HZ :u8 = 0x0D;

pub const BMI270_GYR_BWP_MSK  :u8 = 0x30;
pub const BMI270_GYR_BWP_POS  :u8 = 4;
pub const BMI270_GYR_BWP_OSR4 :u8 = 0x00;
pub const BMI270_GYR_BWP_OSR2 :u8 = 0x01;
pub const BMI270_GYR_BWP_NORM :u8 = 0x02;

pub const BMI270_GYR_FILT_NOISE_MSK      :u8 = 0x40;
pub const BMI270_GYR_FILT_NOISE_POS      :u8 = 6;
pub const BMI270_GYR_FILT_NOISE_PWR      :u8 = 0x00;
pub const BMI270_GYR_FILT_NOISE_PERF     :u8 = 0x01;

pub const BMI270_GYR_FILT_MSK      :u8 = 0x80;
pub const BMI270_GYR_FILT_POS      :u8 = 7;
pub const BMI270_GYR_FILT_PWR_OPT  :u8 = 0x00;
pub const BMI270_GYR_FILT_PERF_OPT :u8 = 0x01;

pub const BMI270_GYR_RANGE_MSK     :u8 = 0x07;
pub const BMI270_GYR_RANGE_2000DPS :u8 = 0x00;
pub const BMI270_GYR_RANGE_1000DPS :u8 = 0x01;
pub const BMI270_GYR_RANGE_500DPS  :u8 = 0x02;
pub const BMI270_GYR_RANGE_250DPS  :u8 = 0x03;
pub const BMI270_GYR_RANGE_125DPS  :u8 = 0x04;

pub const BMI270_GYR_OIS_RANGE_MSK     :u8 = 0x80;
pub const BMI270_GYR_OIS_RANGE_POS     :u8 = 3;
pub const BMI270_GYR_OIS_RANGE_250DPS  :u8 = 0x00;
pub const BMI270_GYR_OIS_RANGE_2000DPS :u8 = 0x01;
/*
#define BMI270_SET_BITS(reg_data, bitname, data)		  \
	((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) \
					  & bitname##_MSK))
#define BMI270_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))
*/

#[macro_export]
macro_rules! BMI270_SET_BITS {
	($reg_data:expr, $bitmask:expr, $bitpos:expr, $data:expr) => {
		(($reg_data & !($bitmask)) | (($data << $bitpos) & $bitmask))
	};
}
#[macro_export]
macro_rules! BMI270_SET_BITS_POS_0 {
	($reg_data:expr, $bitmask:expr, $data:expr) => {
		(($reg_data & !($bitmask)) | ($data & $bitmask))
	};
}
pub struct bmi270_data {
	pub ax :i16 , pub ay: i16, pub az :i16, pub gx : i16, pub gy :i16, pub gz :i16,
	pub acc_range :u8, pub acc_odr :u8, pub gyr_odr :u8,
	pub gyr_range :u16
}

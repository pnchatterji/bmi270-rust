/*
 * Copyright (c) 2024 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #![allow(non_camel_case_types)]
use crate::*;
use heapless::Vec;

const BMI270_WR_LEN :usize = 256;
const BMI270_CONFIG_FILE_RETRIES :u8 = 15;
const BMI270_CONFIG_FILE_POLL_PERIOD_MS :u32 =10;
const BMI270_INTER_WRITE_DELAY_US :u32 = 1000;


pub struct bmi270_feature_reg {
	/* Which feature page the register resides in */
	pub page :u8,
	pub addr :u8
}

pub struct bmi270_feature_config {
	pub name : &'static str,
	pub config_file :&'static [u8],
	pub anymo_1 :bmi270_feature_reg,
	pub anymo_2 :bmi270_feature_reg,
}

pub struct bmi270_config<'a> {
	pub bus :&'a mut dyn SensorBus,
	pub feature : bmi270_feature_config,
}

/* TODO: Required? 
const BMI270_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
const BMI270_SPI_ACC_DELAY_US 2
extern const struct bmi270_bus_io bmi270_bus_io_spi;
*/

pub struct bmi270_device<'a> {
	pub data :bmi270_data,
	pub config :bmi270_config<'a>,
} 

impl<'a> bmi270_device<'a> {

fn bmi270_bus_check(&self) 
	-> Result<(), sensor::ErrorKind> {
	self.config.bus.check()
}

fn bmi270_bus_init(&mut self) 
	-> Result<(), sensor::ErrorKind>  {
	info!("Initlizing BMI270 Bus");
	self.config.bus.init()
}

fn bmi270_reg_read(&mut self, reg :u8, data : &mut[u8]) 
	-> Result<(), sensor::ErrorKind>  {
	self.config.bus.read(reg, data)
}

fn bmi270_reg_read_byte(&mut self, reg :u8) 
	-> Result<u8, sensor::ErrorKind>  {
	let mut data : [u8;1] = [0]; 
	self.config.bus.read(reg,&mut data)?;
	Ok(data[0])
}

fn bmi270_reg_write(&mut self, reg :u8, data : &[u8]) 
	-> Result<(), sensor::ErrorKind>  {
	self.config.bus.write(reg, data)
}

fn bmi270_reg_write_byte(&mut self, reg :u8, data : u8) 
	-> Result<(), sensor::ErrorKind>  {
	let databuf : [u8;1] = [data]; 
	self.config.bus.write(reg, &databuf)
}

fn bmi270_reg_write_with_delay(&mut self,reg : u8, data : &[u8], delay_us : u32) 
	-> Result<(), sensor::ErrorKind> {
	self.bmi270_reg_write(reg, data)?;
	k_usleep(delay_us);
	Ok(())
}
fn bmi270_reg_write_byte_with_delay(&mut self,reg : u8, data : u8, delay_us : u32) 
	-> Result<(), sensor::ErrorKind> {
	let databuf : [u8;1] = [data]; 
	self.bmi270_reg_write(reg, & databuf)?;
	k_usleep(delay_us);
	Ok(())
}

fn channel_accel_convert(val :&mut sensor_value, raw_val_: i16, range :u8) {
	/* 16 bit accelerometer. 2^15 bits represent the range in G */
	/* Converting from G to m/s^2 */
	let raw_val: i64 = (raw_val_ as i64 * SENSOR_G * range as i64) / i16::MAX as i64;
	val.val1 = (raw_val / 1000000i64) as i32;
	val.val2 = (raw_val % 1000000i64) as i32;
}

fn channel_gyro_convert( val :&mut sensor_value, raw_val_ :i16, range :u16) {
	/* 16 bit gyroscope. 2^15 bits represent the range in degrees/s */
	/* Converting from degrees/s to radians/s */
	let raw_val :i64 = raw_val_ as i64;
	val.val1 = (((raw_val * range as i64* SENSOR_PI)
		     / (180i64 * i16::MAX as i64)) / 1000000i64) as i32;
	val.val2 = (((raw_val * range as i64* SENSOR_PI)
		     / (180i64 * i16::MAX as i64)) % 1000000i64) as i32;
}

fn acc_odr_to_reg(val : &sensor_value) ->u8 {
	let odr :f64 = sensor_value_to_double(val);
	match odr {
		x if x < 1.5625
			=> BMI270_ACC_ODR_25D32_HZ,
		x if x  < 3.125
			=> BMI270_ACC_ODR_25D16_HZ,
		x if x  < 6.25
			=> BMI270_ACC_ODR_25D8_HZ,
		x if x  < 12.5
			=> BMI270_ACC_ODR_25D4_HZ,
		x if x  < 25.0
			=> BMI270_ACC_ODR_25D2_HZ,
		x if x  < 50.0
			=> BMI270_ACC_ODR_25_HZ,
		x if x   < 100.0
			=> BMI270_ACC_ODR_50_HZ,
		x if x  < 200.0
			=> BMI270_ACC_ODR_100_HZ,
		x if x  < 400.0
			=> BMI270_ACC_ODR_200_HZ,
		x if x  < 800.0
			=> BMI270_ACC_ODR_400_HZ,
		x if x  < 1600.0
			=> BMI270_ACC_ODR_800_HZ,
		_
			=> BMI270_ACC_ODR_1600_HZ,
	}
}

fn set_accel_odr_osr(&mut self, odr : Option<&sensor_value>,
			     			osr : Option<&sensor_value>) 
							-> Result<(), sensor::ErrorKind> 
{
	let mut acc_conf = self.bmi270_reg_read_byte(BMI270_REG_ACC_CONF)?;
	let mut pwr_ctrl = self.bmi270_reg_read_byte(BMI270_REG_PWR_CTRL)?;
	let data :&mut bmi270_data = &mut self.data;

	if let Some(odr_) = odr{
		let odr_bits = Self::acc_odr_to_reg(odr_);
		acc_conf = BMI270_SET_BITS_POS_0!(acc_conf, BMI270_ACC_ODR_MSK , odr_bits);
		/* If odr_bits is 0, implies that the sampling frequency is 0Hz
		 * or invalid too.
		 */
		if odr_bits !=0 {
			pwr_ctrl |= BMI270_PWR_CTRL_ACC_EN;
		} else {
			pwr_ctrl &= !BMI270_PWR_CTRL_ACC_EN;
		}

		/* If the Sampling frequency (odr) >= 100Hz, enter performance
		 * mode else, power optimized. This also has a consequence
		 * for the OSR
		 */
		if odr_bits >= BMI270_ACC_ODR_100_HZ {
			acc_conf = BMI270_SET_BITS!(acc_conf, BMI270_ACC_FILT_MSK, BMI270_ACC_FILT_POS,
						   BMI270_ACC_FILT_PERF_OPT);
		} else {
			acc_conf = BMI270_SET_BITS!(acc_conf, BMI270_ACC_FILT_MSK, BMI270_ACC_FILT_POS,
						   BMI270_ACC_FILT_PWR_OPT);
		}

		data.acc_odr = odr_bits;
	}

	if let Some(_osr) = osr {
		let osr_bits: u8 = 
			if data.acc_odr >= BMI270_ACC_ODR_100_HZ {
				/* Performance mode */
				/* osr->val2 should be unused */
				match _osr.val1 {
				4 => BMI270_ACC_BWP_OSR4_AVG1,
				2 => BMI270_ACC_BWP_OSR2_AVG2,
				1 => BMI270_ACC_BWP_NORM_AVG4,
				_ => BMI270_ACC_BWP_CIC_AVG8,
				}
			} else {
				/* Power optimized mode */
				/* osr->val2 should be unused */
				match _osr.val1 {
					1 =>BMI270_ACC_BWP_OSR4_AVG1,
					2 =>BMI270_ACC_BWP_OSR2_AVG2,
					4 =>BMI270_ACC_BWP_NORM_AVG4,
					8 =>BMI270_ACC_BWP_CIC_AVG8,
					16 =>BMI270_ACC_BWP_RES_AVG16,
					32 =>BMI270_ACC_BWP_RES_AVG32,
					64 =>BMI270_ACC_BWP_RES_AVG64,
					128 =>BMI270_ACC_BWP_RES_AVG128,
					_ => return Err(sensor::ErrorKind::Unsupported)
				}
			};

		acc_conf = BMI270_SET_BITS!(acc_conf, BMI270_ACC_BWP_MSK,BMI270_ACC_BWP_POS,
					   osr_bits);
	}

	self.bmi270_reg_write_byte(BMI270_REG_ACC_CONF, acc_conf)?;

	/* Assuming we have advance power save enabled */
	k_usleep(BMI270_TRANSC_DELAY_SUSPEND);

	pwr_ctrl &= BMI270_PWR_CTRL_MSK;
	self.bmi270_reg_write_byte_with_delay(BMI270_REG_PWR_CTRL,
						pwr_ctrl, 
						BMI270_INTER_WRITE_DELAY_US)?;

	Ok(())
}

fn set_accel_range(&mut self, range :& sensor_value)
	-> Result<(), sensor::ErrorKind> {
	let mut acc_range: u8 = self.bmi270_reg_read_byte(BMI270_REG_ACC_RANGE)?;
	let data :&mut bmi270_data = &mut self.data;
	let reg:u8;


	/* range->val2 is unused */
	match range.val1 {
		2 => {
			reg = BMI270_ACC_RANGE_2G;
			data.acc_range = 2;
		},
		4 =>{
			reg = BMI270_ACC_RANGE_4G;
			data.acc_range = 4;
		},
		8 =>{
			reg = BMI270_ACC_RANGE_8G;
			data.acc_range = 8;
		},
		16 =>{
			reg = BMI270_ACC_RANGE_16G;
			data.acc_range = 16;
		},
		_ =>	return Err(sensor::ErrorKind::Unsupported)
	};

	acc_range = BMI270_SET_BITS_POS_0!(acc_range, BMI270_ACC_RANGE_MSK,
					  reg);
	self.bmi270_reg_write_byte_with_delay(BMI270_REG_ACC_RANGE, 
		acc_range,  BMI270_INTER_WRITE_DELAY_US)?;

	Ok(())
}

fn gyr_odr_to_reg(val : &sensor_value) ->u8
{
	let odr :f64 = sensor_value_to_double(val);
	match odr {
		x if x < 50.0 => BMI270_GYR_ODR_25_HZ,
		x if x < 100.0 => BMI270_GYR_ODR_50_HZ,
		x if x < 200.0 => BMI270_GYR_ODR_100_HZ,
		x if x < 400.0 => BMI270_GYR_ODR_200_HZ,
		x if x < 800.0 => BMI270_GYR_ODR_400_HZ,
		x if x < 1600.0 => BMI270_GYR_ODR_800_HZ,
		x if x < 3200.0 => BMI270_GYR_ODR_1600_HZ,
		_ => BMI270_GYR_ODR_3200_HZ
	}
}

fn set_gyro_odr_osr(&mut self, 	odr :Option<&sensor_value>, 
							osr :Option<&sensor_value>
			)-> Result<(), sensor::ErrorKind> {
	let mut gyr_conf: u8 = self.bmi270_reg_read_byte(BMI270_REG_GYR_CONF)?;
	let mut pwr_ctrl: u8 = self.bmi270_reg_read_byte(BMI270_REG_PWR_CTRL)?;
	let data :&mut bmi270_data =&mut self.data;	

	if let Some(odr_) = odr {
		let odr_bits:u8 = Self::gyr_odr_to_reg(&odr_);
		gyr_conf = BMI270_SET_BITS_POS_0!(gyr_conf, BMI270_GYR_ODR_MSK,
						 odr_bits);

		/* If odr_bits is 0, implies that the sampling frequency is
		 * 0Hz or invalid too.
		 */
		if odr_bits !=0 {
			pwr_ctrl |= BMI270_PWR_CTRL_GYR_EN;
		} else {
			pwr_ctrl &= !BMI270_PWR_CTRL_GYR_EN;
		}

		/* If the Sampling frequency (odr) >= 100Hz, enter performance
		 * mode else, power optimized. This also has a consequence for
		 * the OSR
		 */
		if odr_bits >= BMI270_GYR_ODR_100_HZ {
			gyr_conf = BMI270_SET_BITS!(gyr_conf,
						   BMI270_GYR_FILT_MSK,BMI270_GYR_FILT_POS,
						   BMI270_GYR_FILT_PERF_OPT);
			gyr_conf = BMI270_SET_BITS!(gyr_conf,
						   BMI270_GYR_FILT_NOISE_MSK,BMI270_GYR_FILT_NOISE_POS,
						   BMI270_GYR_FILT_NOISE_PERF);
		} else {
			gyr_conf = BMI270_SET_BITS!(gyr_conf,
						   BMI270_GYR_FILT_MSK,BMI270_GYR_FILT_POS,
						   BMI270_GYR_FILT_PWR_OPT);
			gyr_conf = BMI270_SET_BITS!(gyr_conf,
						   BMI270_GYR_FILT_NOISE_MSK,BMI270_GYR_FILT_NOISE_POS,
						   BMI270_GYR_FILT_NOISE_PWR);
		}

		data.gyr_odr = odr_bits;
	}

	if let Some(osr_) = osr {
		/* osr->val2 should be unused */
		let osr_bits: u8 = match osr_.val1 {
			4=> BMI270_GYR_BWP_OSR4,
			2=> BMI270_GYR_BWP_OSR2,
			_ => BMI270_GYR_BWP_NORM
		};

		gyr_conf = BMI270_SET_BITS!(gyr_conf, BMI270_GYR_BWP_MSK,BMI270_GYR_BWP_POS,
					   osr_bits);
	}

		self.bmi270_reg_write_byte(BMI270_REG_GYR_CONF, gyr_conf)?;

		/* Assuming we have advance power save enabled */
		k_usleep(BMI270_TRANSC_DELAY_SUSPEND);

		pwr_ctrl &= BMI270_PWR_CTRL_MSK;
		self.bmi270_reg_write_byte_with_delay(BMI270_REG_PWR_CTRL,
						  pwr_ctrl, BMI270_INTER_WRITE_DELAY_US)?;
		Ok(())
}

fn set_gyro_range(&mut self,range :&sensor_value)
	-> Result<(), sensor::ErrorKind> {
	let mut gyr_range :u8 = self.bmi270_reg_read_byte(BMI270_REG_GYR_RANGE)?;
	let data : &mut bmi270_data = &mut self.data;
	let reg:u8;

	/* range->val2 is unused */
	match range.val1 {
		125 => {
			reg = BMI270_GYR_RANGE_125DPS;
			data.gyr_range = 125;
		},
		250 =>{
			reg = BMI270_GYR_RANGE_250DPS;
			data.gyr_range = 250;
		},
		500=>{
			reg = BMI270_GYR_RANGE_500DPS;
			data.gyr_range = 500;
		},
		1000 =>{
			reg = BMI270_GYR_RANGE_1000DPS;
			data.gyr_range = 1000;
		},
		2000 =>{
			reg = BMI270_GYR_RANGE_2000DPS;
			data.gyr_range = 2000;
		},
		_ =>
			return Err(sensor::ErrorKind::Unsupported)
	}

	gyr_range = BMI270_SET_BITS_POS_0!(gyr_range, BMI270_GYR_RANGE_MSK, reg);
	self.bmi270_reg_write_byte_with_delay(BMI270_REG_GYR_RANGE, gyr_range,
					   BMI270_INTER_WRITE_DELAY_US)?;
	Ok(())
}

fn write_config_file(&mut self)
	-> Result<(), sensor::ErrorKind> {
//	let cfg :&mut bmi270_config = &mut self.config;
	let mut index :usize = 0;
	let mut addr_array :[u8;2] = [ 0 , 0 ];

	info!("writing config file {}", self.config.feature.name);

	/* Disable loading of the configuration */
	while index <self.config.feature.config_file.len() {
		/* Store 0 to 3 bits of address in first byte */
		addr_array[0] = ((index / 2) & 0x0F) as u8;

		/* Store 4 to 11 bits of address in the second byte */
		addr_array[1] = ((index / 2) >> 4) as u8;
		let remaining: usize = self.config.feature.config_file.len() - index;

		let ssize: usize = if remaining > BMI270_WR_LEN {BMI270_WR_LEN} else {remaining};
		let iend: usize = index+ssize;
		let fslice = &self.config.feature.config_file[index..iend];
		self.bmi270_reg_write_with_delay( BMI270_REG_INIT_ADDR_0,
						  &addr_array, BMI270_INTER_WRITE_DELAY_US)?;
		self.bmi270_reg_write_with_delay(
					BMI270_REG_INIT_DATA,
					fslice,
					BMI270_INTER_WRITE_DELAY_US)?;
		index += BMI270_WR_LEN;
	}

	Ok(())
}

fn bmi270_sample_fetch(&mut self, chan :sensor_channel)
-> Result<(), sensor::ErrorKind> {
	if chan != SENSOR_CHAN_ALL {
		return Err(sensor::ErrorKind::Unsupported);
	}
	let mut buf : [u8;12] =[0;12];
	let ret = self.bmi270_reg_read(BMI270_REG_ACC_X_LSB, &mut buf);
	let data :&mut bmi270_data = &mut self.data;
	if ret.is_ok() {
		data.ax = i16::from_le_bytes(buf[0..2].try_into().unwrap());
		data.ay = i16::from_le_bytes(buf[2..4].try_into().unwrap());
		data.az = i16::from_le_bytes(buf[4..6].try_into().unwrap());
		data.gx = i16::from_le_bytes(buf[6..8].try_into().unwrap());
		data.gy = i16::from_le_bytes(buf[8..10].try_into().unwrap());
		data.gz = i16::from_le_bytes(buf[10..12].try_into().unwrap());
	} else {
		data.ax = 0;
		data.ay = 0;
		data.az = 0;
		data.gx = 0;
		data.gy = 0;
		data.gz = 0;
	}
	ret
}

fn bmi270_channel_get(&self,chan: sensor_channel, val :&mut Vec<sensor_value,3>)
	  -> Result<(), sensor::ErrorKind> {
	let data : &bmi270_data = &self.data;
    match chan {
		SENSOR_CHAN_ACCEL_X =>
			Self::channel_accel_convert(&mut val[0], data.ax, data.acc_range),
		SENSOR_CHAN_ACCEL_Y =>
			Self::channel_accel_convert(&mut val[0], data.ay, data.acc_range),
		SENSOR_CHAN_ACCEL_Z =>
			Self::channel_accel_convert(&mut val[0], data.az, data.acc_range),
		SENSOR_CHAN_ACCEL_XYZ => {
			Self::channel_accel_convert(&mut val[0], data.ax,data.acc_range);
			Self::channel_accel_convert(&mut val[1], data.ay,data.acc_range);
			Self::channel_accel_convert(&mut val[2], data.az,data.acc_range);
			},
		SENSOR_CHAN_GYRO_X =>
			Self::channel_gyro_convert(&mut val[0], data.gx, data.gyr_range),
		SENSOR_CHAN_GYRO_Y =>
			Self::channel_gyro_convert(&mut val[0], data.gy, data.gyr_range),
		SENSOR_CHAN_GYRO_Z => 
			Self::channel_gyro_convert(&mut val[0], data.gz, data.gyr_range),
		SENSOR_CHAN_GYRO_XYZ => {
			Self::channel_gyro_convert(&mut val[0], data.gx, data.gyr_range);
			Self::channel_gyro_convert(&mut val[1], data.gy, data.gyr_range);
			Self::channel_gyro_convert(&mut val[2], data.gz, data.gyr_range);
			},
		_ => {
				return Err(sensor::ErrorKind::Unsupported);
			}
	}
	Ok(())
}
/*
#if defined(CONFIG_BMI270_TRIGGER)

/* ANYMO_1.duration conversion is 20 ms / LSB */
#define ANYMO_1_DURATION_MSEC_TO_LSB(_ms)	\
	BMI270_ANYMO_1_DURATION(_ms / 20)

static int bmi270_write_anymo_threshold(const struct device *dev,
					struct sensor_value val)
{
	struct bmi270_data *data = dev->data;

	/* this takes configuration in g. */
	if (val.val1 > 0) {
		LOG_DBG("anymo_threshold set to max");
		val.val2 = 1e6;
	}

	/* max = BIT_MASK(10) = 1g => 0.49 mg/LSB */
	uint16_t lsbs = (val.val2 * BMI270_ANYMO_2_THRESHOLD_MASK) / 1e6;

	if (!lsbs) {
		LOG_ERR("Threshold too low!");
		return -EINVAL;
	}

	uint16_t anymo_2 = BMI270_ANYMO_2_THRESHOLD(lsbs)
		| BMI270_ANYMO_2_OUT_CONF_BIT_6;

	data->anymo_2 = anymo_2;
	return 0;
}

static int bmi270_write_anymo_duration(const struct device *dev, uint32_t ms)
{
	struct bmi270_data *data = dev->data;
	uint16_t val = ANYMO_1_DURATION_MSEC_TO_LSB(ms)
		| BMI270_ANYMO_1_SELECT_XYZ;

	data->anymo_1 = val;
	return 0;
}
#endif /* CONFIG_BMI270_TRIGGER */
*/
fn bmi270_attr_set(&mut self, chan :sensor_channel,
			   attr :sensor_attribute, val : & sensor_value)
		   -> Result<(), sensor::ErrorKind> {

	match chan {
		SENSOR_CHAN_ACCEL_X | SENSOR_CHAN_ACCEL_Y| SENSOR_CHAN_ACCEL_Z| SENSOR_CHAN_ACCEL_XYZ => {
			match attr {
			SENSOR_ATTR_SAMPLING_FREQUENCY=> self.set_accel_odr_osr(Some(val), None)?,		
			SENSOR_ATTR_OVERSAMPLING =>self.set_accel_odr_osr(None,Some(val))?,
			SENSOR_ATTR_FULL_SCALE=> self.set_accel_range(val)?,
			// #if defined(CONFIG_BMI270_TRIGGER)
			// 		case SENSOR_ATTR_SLOPE_DUR:
			// 			return bmi270_write_anymo_duration(dev, val->val1);
			// 		case SENSOR_ATTR_SLOPE_TH:
			// 			return bmi270_write_anymo_threshold(dev, *val);
			// #endif
			_ => {return Err(sensor::ErrorKind::Unsupported);},
			};
		},
		SENSOR_CHAN_GYRO_X | SENSOR_CHAN_GYRO_Y| SENSOR_CHAN_GYRO_Z| SENSOR_CHAN_GYRO_XYZ => {
			match attr {
			SENSOR_ATTR_SAMPLING_FREQUENCY => self.set_gyro_odr_osr(Some(val), None)?,
			SENSOR_ATTR_OVERSAMPLING => self.set_gyro_odr_osr(None, Some(val))?,
			SENSOR_ATTR_FULL_SCALE =>self.set_gyro_range(val)?,
			_ => {return Err(sensor::ErrorKind::Unsupported);},
			};
		},
		_ => {return Err(sensor::ErrorKind::Unsupported);},
	};

	Ok(())
}

fn bmi270_init(&mut self)
	-> Result<(), sensor::ErrorKind> {
	info!("Initializing BMI270");
	self.config.bus.init()?;
	self.bmi270_bus_check()?;
	let data : &mut bmi270_data = &mut self.data;

// #if CONFIG_BMI270_TRIGGER
// 	data->dev = dev;
// 	k_mutex_init(&data->trigger_mutex);
// #endif

	data.acc_odr = BMI270_ACC_ODR_100_HZ;
	data.acc_range = 8;
	data.gyr_odr = BMI270_GYR_ODR_200_HZ;
	data.gyr_range = 2000;
	
//	k_msleep(BMI270_POWER_ON_TIME);

	self.bmi270_bus_init()?;

	let chip_id: u8 = self.bmi270_reg_read_byte(BMI270_REG_CHIP_ID)?;

	if chip_id != BMI270_CHIP_ID {
		error!("Unexpected chip id {}. Expected {}",
			chip_id, BMI270_CHIP_ID);
		return Err(sensor::ErrorKind::InvalidData);
	}

	self.bmi270_reg_write_byte(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET)?;

	k_usleep(BMI270_SOFT_RESET_TIME);

	let mut adv_pwr_save = self.bmi270_reg_read_byte(BMI270_REG_PWR_CONF)?;

	adv_pwr_save = BMI270_SET_BITS_POS_0!(adv_pwr_save,
					     BMI270_PWR_CONF_ADV_PWR_SAVE_MSK,
					     BMI270_PWR_CONF_ADV_PWR_SAVE_DIS);
	self.bmi270_reg_write_byte_with_delay(BMI270_REG_PWR_CONF,
					  adv_pwr_save,
					  BMI270_INTER_WRITE_DELAY_US)?;
	self.bmi270_reg_write_byte(BMI270_REG_INIT_CTRL, BMI270_PREPARE_CONFIG_LOAD)?;
	self.write_config_file()?;

	self.bmi270_reg_write_byte(BMI270_REG_INIT_CTRL, BMI270_COMPLETE_CONFIG_LOAD)?;

	/* Timeout after BMI270_CONFIG_FILE_RETRIES x
	 * BMI270_CONFIG_FILE_POi64_PERIOD_US microseconds.
	 * If tries is BMI270_CONFIG_FILE_RETRIES by the end of the loop,
	 * report an error
	 */
	 let mut tries = 0;
	 while tries < BMI270_CONFIG_FILE_RETRIES {
		let mut msg: u8 = self.bmi270_reg_read_byte(BMI270_REG_INTERNAL_STATUS)?;

		msg &= BMI270_INST_MESSAGE_MSK;
		if msg == BMI270_INST_MESSAGE_INIT_OK {
			break;
		}

		k_msleep(BMI270_CONFIG_FILE_POLL_PERIOD_MS);
		tries+=1;
	}

	if tries >= BMI270_CONFIG_FILE_RETRIES {
		return Err(sensor::ErrorKind::TimedOut);
	}

// #if CONFIG_BMI270_TRIGGER
// 	ret = bmi270_init_interrupts(dev);
// 	if (ret) {
// 		LOG_ERR("bmi270_init_interrupts returned %d", ret);
// 		return ret;
// 	}
// #endif

	adv_pwr_save = BMI270_SET_BITS_POS_0!(adv_pwr_save,
					     BMI270_PWR_CONF_ADV_PWR_SAVE_MSK,
					     BMI270_PWR_CONF_ADV_PWR_SAVE_EN);
	self.bmi270_reg_write_byte_with_delay(BMI270_REG_PWR_CONF,
					  adv_pwr_save,
					  BMI270_INTER_WRITE_DELAY_US)?;

	Ok(())
}

pub fn new(sbus: &'a mut dyn SensorBus) -> Self {
	bmi270_device{
		data : bmi270_data {
			ax:0,ay:0,az:0, 
			gx:0,gy: 0,gz: 0,
			acc_range: 0,acc_odr: 0,
			gyr_odr: 0,	gyr_range: 0,
		},
		config : bmi270_config {
			bus: sbus,
			feature: bmi270_feature_config {
				name: "foo",
				config_file: &BMI270_CONFIG_FILE_BASE,
				anymo_1: bmi270_feature_reg { page: 0, addr: 0 },
				anymo_2: bmi270_feature_reg { page: 0, addr: 0 },
			},
		},
	}
}

}


impl<'a> SensorDevice for bmi270_device<'a> {

	fn name (&self) -> &str {
		"bmi270" 
	}

	fn device_is_ready (&self) 
	-> bool {
		true  //TODO: check if init
	}

	fn init (&mut self)					
		-> Result<(), ErrorKind> {
			self.bmi270_init()
	}

	fn sensor_attr_set (&mut self, 
		chan :sensor_channel, attr :sensor_attribute, val : &sensor_value) 
		-> Result<(), ErrorKind> {
			self.bmi270_attr_set(chan, attr, val)
	}

	fn sensor_attr_get ( &self,
		_chan :sensor_channel, _attr :sensor_attribute, _val : &sensor_value)
		-> Result<(), ErrorKind> {
			Err(sensor::ErrorKind::Unsupported)
	}

	fn sensor_trigger_set ( &self,
		_trig : &sensor_trigger,	_handler :sensor_trigger_handler_t)
		-> Result<(), ErrorKind> {
			Err(sensor::ErrorKind::Unsupported)
	}

	fn sensor_sample_fetch_chan(&mut self,
		chan: sensor_channel)
		-> Result<(), ErrorKind> {
			self.bmi270_sample_fetch(chan)
	}
	
	fn sensor_channel_get(&self,
		chan :sensor_channel, val :&mut Vec<sensor_value,3>)
		-> Result<(), ErrorKind> {
		self.bmi270_channel_get(chan, val)
	}
}
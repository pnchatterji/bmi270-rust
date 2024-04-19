/*
 * Copyright (c) 2024 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #![no_std]
 #![no_main]
use bmi270_defs::*;
use bmi270::*;
use defmt::*;
use sensor::*;
use app30_spi::*;

use crate::sensor_channel::*;
use crate::sensor_attribute::*;
use crate::bmi270_config_file::*;
use embassy_executor::Spawner;
use heapless::Vec;

use {defmt_rtt as _, panic_probe as _};

pub mod bmi270;
pub mod bmi270_defs;
pub mod sensor;
pub mod bmi270_config_file;
pub mod app30_spi;
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    accel_main().unwrap_or_else(|err:sensor::ErrorKind|
		println!("Error {}",err));
}

fn accel_main() 
-> Result<(), sensor::ErrorKind> {
	let sbus:&mut dyn SensorBus = &mut app30_spi_bus::new();
	let dev : &mut dyn SensorDevice = &mut bmi270_device::new(sbus);
	let mut acc :Vec<sensor_value,3> = Vec::from_slice(
			&[sensor_value{val1:0,val2:0},
			sensor_value{val1:0,val2:0},
			sensor_value{val1:0,val2:0}]).unwrap();
	let mut gyr :Vec<sensor_value,3> = Vec::from_slice(
		&[sensor_value{val1:0,val2:0},
		sensor_value{val1:0,val2:0},
		sensor_value{val1:0,val2:0}]).unwrap();
	
	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	 let mut full_scale :sensor_value = sensor_value{
				val1 : 2,            /* G */
				val2 : 0,
			};
	let mut sampling_freq :sensor_value = sensor_value{
		val1 : 100,       /* Hz. Performance mode */
		val2 : 0,
		};
	let mut oversampling :sensor_value = sensor_value {
			val1 : 1,          /* Normal mode */
			val2 : 0,
		};
	dev.init()?;

	if !dev.device_is_ready() {
		println!("Device {} is not ready", dev.name());
		return Ok(());
	}


	println!("Device name is {}",dev.name());


	dev.sensor_attr_set(SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale)?;
	dev.sensor_attr_set(SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling)?;
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	dev.sensor_attr_set(SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq)?;


	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = 500;          /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;       /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;          /* Normal mode */
	oversampling.val2 = 0;

	dev.sensor_attr_set(SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale)?;
	dev.sensor_attr_set(SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling)?;
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	dev.sensor_attr_set(SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq)?;

	loop {
		k_msleep(3000);

		dev.sensor_sample_fetch()?;

		dev.sensor_channel_get(SENSOR_CHAN_ACCEL_XYZ, &mut acc)?;
		dev.sensor_channel_get(SENSOR_CHAN_GYRO_XYZ, &mut gyr)?;

		println!("Accel: ({},{},{}) Gyr: ({},{},{})\n",
		       acc[0], acc[1], acc[2],
			   gyr[0], gyr[1], gyr[2]);
	}
}

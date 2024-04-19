/*
 * Copyright (c) 2024 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shuttle Board SPI1 on Bosch Sensortec Application Board 3.0
 */
#![allow(non_camel_case_types)]

use defmt::*;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
use embassy_nrf::spim::Spim;
use embassy_nrf::{bind_interrupts, peripherals, spim};
use embedded_hal::delay::DelayNs;
use {defmt_rtt as _, panic_probe as _};
use crate::*; 

bind_interrupts!(struct Irqs {
	SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

type SpimType = Spim<'static, peripherals::SPI3>;
type PinType = Output<'static, AnyPin>;

pub struct app30_spi_bus {
	ncs: PinType,
	vdd: PinType,
	vddio: PinType,
	vddsel: PinType,
	shspi: SpimType,
	initialized:bool,
}

impl app30_spi_bus {
pub fn  new() 
	 -> Self {
	 let p = embassy_nrf::init(Default::default());
	 let mut config = spim::Config::default();
	 config.frequency = spim::Frequency::M8;
	 app30_spi_bus {
	 ncs : Output::new(AnyPin::from(p.P0_24), Level::Low, OutputDrive::Standard),
	 vdd : Output::new(AnyPin::from(p.P0_03), Level::Low, OutputDrive::Standard),
	 vddio :Output::new(AnyPin::from(p.P0_28), Level::Low, OutputDrive::Standard),
	 vddsel :Output::new(AnyPin::from(p.P0_27), Level::Low, OutputDrive::Standard),
	 //sck 0.16, miso 0.15, mosi 0.06
	 shspi : spim::Spim::new(p.SPI3, Irqs, p.P0_16, p.P0_15, p.P0_06, config),
	 initialized: false,
	 }
}
}
impl SensorBus for app30_spi_bus {

fn  init(&mut self) 
	 -> Result<(), sensor::ErrorKind> {
	info!("Initializing APP30 Shuttle SPI Bus");
	// softreset Rx edge on CS
	self.vdd.set_high();
	self.vddio.set_high();
	self.vddsel.set_high();
	k_msleep(800); 
	self.ncs.set_low();
	k_msleep(800); 
	self.ncs.set_high();
	k_msleep(800); 
	self.initialized=true;
	Ok(())
}

fn check(&self) 
	-> Result<(), sensor::ErrorKind> {
	info!("app30_spi_check");
	if self.initialized {Ok(())} else {Err(sensor::ErrorKind::NotConnected)}
}

fn read(&mut self, adr: u8, data: &mut [u8]) 
	-> Result<(), sensor::ErrorKind> {
	info!("app30_spi_read");
	if ! self.initialized {
		return Err(sensor::ErrorKind::NotConnected)
	}
	self.ncs.set_low();
	let tx:[u8;1] = [adr|0x80];
	self.shspi.blocking_write(&tx).unwrap();//TODO: return sensorError instead of panicking!!
	let mut dummy_byte:[u8;1] = [0];
	self.shspi.blocking_read(&mut dummy_byte).unwrap();
	self.shspi.blocking_read(data).unwrap();
	self.ncs.set_high();
	info!("exit app30_spi_read tx {:02x} data {:02x} \n",tx,data);
	Ok(())
}

fn write(&mut self, adr: u8, data: &[u8]) 
	-> Result<(), sensor::ErrorKind>{
		if ! self.initialized {
			return Err(sensor::ErrorKind::NotConnected)
		}
		self.ncs.set_low();
	let tx:[u8;1] = [adr];
	self.shspi.blocking_write(&tx).unwrap();//TODO: return sensorError instead of panicking!!
	self.shspi.blocking_write(data).unwrap();
	self.ncs.set_high();
	Ok(())
}

}

pub fn k_usleep(tim :u32) {
	embassy_time::Delay.delay_us(tim);
}

pub fn k_msleep(tim :u32) {
	embassy_time::Delay.delay_ms(tim);
}

#![allow(non_camel_case_types)]

use heapless::Vec;

/**
 * @file sensor.rs
 *
 * @brief API for accessing sensor drivers.
 * Copyright (c) 2024 Bosch Sensortec GmbH
 * SPDX-License-Identifier: Apache-2.0
 * This file has been adapted from the Zephyr Sensor Driver API C header file (sensor.h). 
 * https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/sensor.h
 * Source copyright is acknowledged.
 */


 /* @brief Representation of a sensor readout value.
 *
 * The value is represented as having an integer and a fractional part,
 * and can be obtained using the formula val1 + val2 * 10^(-6). Negative
 * values also adhere to the above formula, but may need special attention.
 * Here are some examples of the value representation:
 *
 *      0.5: val1 =  0, val2 =  500000
 *     -0.5: val1 =  0, val2 = -500000
 *     -1.0: val1 = -1, val2 =  0
 *     -1.5: val1 = -1, val2 = -500000
 */
#[derive(Clone)]
pub struct sensor_value {
	/** Integer part of the value. */
	pub val1 :i32,
	/** Fractional part of the value (in one-miionth parts). */
	pub val2 :i32
}

impl defmt::Format for sensor_value {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
		let sign = if self.val1<0 || self.val2 <0 {'-'} else {' '};
        defmt::write!(f, "{}{}.{}",
           sign,self.val1.abs(),self.val2.abs());
    }
}
/**
 * @brief Sensor channels.
 */
#[derive(PartialEq)]
pub enum sensor_channel {
	/** Acceleration on the X axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_X,
	/** Acceleration on the Y axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_Y,
	/** Acceleration on the Z axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_Z,
	/** Acceleration on the X, Y and Z axes. */
	SENSOR_CHAN_ACCEL_XYZ,
	/** Angular velocity around the X axis, in radians/s. */
	SENSOR_CHAN_GYRO_X,
	/** Angular velocity around the Y axis, in radians/s. */
	SENSOR_CHAN_GYRO_Y,
	/** Angular velocity around the Z axis, in radians/s. */
	SENSOR_CHAN_GYRO_Z,
	/** Angular velocity around the X, Y and Z axes. */
	SENSOR_CHAN_GYRO_XYZ,
	/** Magnetic field on the X axis, in Gauss. */
	SENSOR_CHAN_MAGN_X,
	/** Magnetic field on the Y axis, in Gauss. */
	SENSOR_CHAN_MAGN_Y,
	/** Magnetic field on the Z axis, in Gauss. */
	SENSOR_CHAN_MAGN_Z,
	/** Magnetic field on the X, Y and Z axes. */
	SENSOR_CHAN_MAGN_XYZ,
	/** Device die temperature in degrees Celsius. */
	SENSOR_CHAN_DIE_TEMP,
	/** Ambient temperature in degrees Celsius. */
	SENSOR_CHAN_AMBIENT_TEMP,
	/** Pressure in kilopascal. */
	SENSOR_CHAN_PRESS,
	/**
	 * Proximity.  Adimensional.  A value of 1 indicates that an
	 * object is close.
	 */
	SENSOR_CHAN_PROX,
	/** Humidity, in percent. */
	SENSOR_CHAN_HUMIDITY,
	/** Iuminance in visible spectrum, in lux. */
	SENSOR_CHAN_LIGHT,
	/** Iuminance in infra-red spectrum, in lux. */
	SENSOR_CHAN_IR,
	/** Iuminance in red spectrum, in lux. */
	SENSOR_CHAN_RED,
	/** Iuminance in green spectrum, in lux. */
	SENSOR_CHAN_GREEN,
	/** Iuminance in blue spectrum, in lux. */
	SENSOR_CHAN_BLUE,
	/** Altitude, in meters */
	SENSOR_CHAN_ALTITUDE,

	/** 1.0 micro-meters Particulate Matter, in ug/m^3 */
	SENSOR_CHAN_PM_1_0,
	/** 2.5 micro-meters Particulate Matter, in ug/m^3 */
	SENSOR_CHAN_PM_2_5,
	/** 10 micro-meters Particulate Matter, in ug/m^3 */
	SENSOR_CHAN_PM_10,
	/** Distance. From sensor to target, in meters */
	SENSOR_CHAN_DISTANCE,

	/** CO2 level, in parts per miion (ppm) **/
	SENSOR_CHAN_CO2,
	/** VOC level, in parts per biion (ppb) **/
	SENSOR_CHAN_VOC,
	/** Gas sensor resistance in ohms. */
	SENSOR_CHAN_GAS_RES,

	/** Voltage, in volts **/
	SENSOR_CHAN_VOLTAGE,

	/** Current Shunt Voltage in mii-volts **/
	SENSOR_CHAN_VSHUNT,

	/** Current, in amps **/
	SENSOR_CHAN_CURRENT,
	/** Power in watts **/
	SENSOR_CHAN_POWER,

	/** Resistance , in Ohm **/
	SENSOR_CHAN_RESISTANCE,

	/** Angular rotation, in degrees */
	SENSOR_CHAN_ROTATION,

	/** Position change on the X axis, in points. */
	SENSOR_CHAN_POS_DX,
	/** Position change on the Y axis, in points. */
	SENSOR_CHAN_POS_DY,
	/** Position change on the Z axis, in points. */
	SENSOR_CHAN_POS_DZ,

	/** Revolutions per minute, in RPM. */
	SENSOR_CHAN_RPM,

	/** Voltage, in volts **/
	SENSOR_CHAN_GAUGE_VOLTAGE,
	/** Average current, in amps **/
	SENSOR_CHAN_GAUGE_AVG_CURRENT,
	/** Standby current, in amps **/
	SENSOR_CHAN_GAUGE_STDBY_CURRENT,
	/** Max load current, in amps **/
	SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT,
	/** Gauge temperature  **/
	SENSOR_CHAN_GAUGE_TEMP,
	/** State of charge measurement in % **/
	SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
	/** Fu Charge Capacity in mAh **/
	SENSOR_CHAN_GAUGE_FU_CHARGE_CAPACITY,
	/** Remaining Charge Capacity in mAh **/
	SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY,
	/** Nominal Available Capacity in mAh **/
	SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY,
	/** Fu Available Capacity in mAh **/
	SENSOR_CHAN_GAUGE_FU_AVAIL_CAPACITY,
	/** Average power in mW **/
	SENSOR_CHAN_GAUGE_AVG_POWER,
	/** State of health measurement in % **/
	SENSOR_CHAN_GAUGE_STATE_OF_HEALTH,
	/** Time to empty in minutes **/
	SENSOR_CHAN_GAUGE_TIME_TO_EMPTY,
	/** Time to fu in minutes **/
	SENSOR_CHAN_GAUGE_TIME_TO_FU,
	/** Cycle count (total number of charge/discharge cycles) **/
	SENSOR_CHAN_GAUGE_CYCLE_COUNT,
	/** Design voltage of ce in V (max voltage)*/
	SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE,
	/** Desired voltage of ce in V (nominal voltage) */
	SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE,
	/** Desired charging current in mA */
	SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,

	/** A channels. */
	SENSOR_CHAN_ALL,

	/**
	 * Number of a common sensor channels.
	 */
	SENSOR_CHAN_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	SENSOR_CHAN_PRIV_START,

	/**
	 * Maximum value describing a sensor channel type.
	 */
	SENSOR_CHAN_MAX,
}

/**
 * @brief Sensor trigger types.
 */
#[derive(PartialEq)]
pub enum sensor_trigger_type {
	/**
	 * Timer-based trigger, useful when the sensor does not have an
	 * interrupt line.
	 */
	SENSOR_TRIG_TIMER,
	/** Trigger fires whenever new data is ready. */
	SENSOR_TRIG_DATA_READY,
	/**
	 * Trigger fires when the selected channel varies significantly.
	 * This includes any-motion detection when the channel is
	 * acceleration or gyro. If detection is based on slope between
	 * successive channel readings, the slope threshold is configured
	 * via the @ref SENSOR_ATTR_SLOPE_TH and @ref SENSOR_ATTR_SLOPE_DUR
	 * attributes.
	 */
	SENSOR_TRIG_DELTA,
	/** Trigger fires when a near/far event is detected. */
	SENSOR_TRIG_NEAR_FAR,
	/**
	 * Trigger fires when channel reading transitions configured
	 * thresholds.  The thresholds are configured via the @ref
	 * SENSOR_ATTR_LOWER_THRESH, @ref SENSOR_ATTR_UPPER_THRESH, and
	 * @ref SENSOR_ATTR_HYSTERESIS attributes.
	 */
	SENSOR_TRIG_THRESHOLD,

	/** Trigger fires when a single tap is detected. */
	SENSOR_TRIG_TAP,

	/** Trigger fires when a double tap is detected. */
	SENSOR_TRIG_DOUBLE_TAP,

	/** Trigger fires when a free fa is detected. */
	SENSOR_TRIG_FREEFA,

	/** Trigger fires when motion is detected. */
	SENSOR_TRIG_MOTION,

	/** Trigger fires when no motion has been detected for a while. */
	SENSOR_TRIG_STATIONARY,
	/**
	 * Number of a common sensor triggers.
	 */
	SENSOR_TRIG_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	SENSOR_TRIG_PRIV_START,

	/**
	 * Maximum value describing a sensor trigger type.
	 */
	SENSOR_TRIG_MAX,
}

/**
 * @brief Sensor trigger spec.
 */
pub struct sensor_trigger {
	/** Trigger type. */
	pub sens_t :sensor_trigger_type,
	/** Channel the trigger is set on. */
	pub chan: sensor_channel,
}

/**
 * @brief Sensor attribute types.
 */
#[derive(PartialEq)]
pub enum sensor_attribute {
	/**
	 * Sensor sampling frequency, i.e. how many times a second the
	 * sensor takes a measurement.
	 */
	SENSOR_ATTR_SAMPLING_FREQUENCY,
	/** Lower threshold for trigger. */
	SENSOR_ATTR_LOWER_THRESH,
	/** Upper threshold for trigger. */
	SENSOR_ATTR_UPPER_THRESH,
	/** Threshold for any-motion (slope) trigger. */
	SENSOR_ATTR_SLOPE_TH,
	/**
	 * Duration for which the slope values needs to be
	 * outside the threshold for the trigger to fire.
	 */
	SENSOR_ATTR_SLOPE_DUR,
	/* Hysteresis for trigger thresholds. */
	SENSOR_ATTR_HYSTERESIS,
	/** Oversampling factor */
	SENSOR_ATTR_OVERSAMPLING,
	/** Sensor range, in SI units. */
	SENSOR_ATTR_FULL_SCALE,
	/**
	 * The sensor value returned wi be altered by the amount indicated by
	 * offset: final_value = sensor_value + offset.
	 */
	SENSOR_ATTR_OFFSET,
	/**
	 * Calibration target. This wi be used by the internal chip's
	 * algorithms to calibrate itself on a certain axis, or a of them.
	 */
	SENSOR_ATTR_CALIB_TARGET,
	/** Configure the operating modes of a sensor. */
	SENSOR_ATTR_CONFIGURATION,
	/** Set a calibration value needed by a sensor. */
	SENSOR_ATTR_CALIBRATION,
	/** Enable/disable sensor features */
	SENSOR_ATTR_FEATURE_MASK,
	/** Alert threshold or alert enable/disable */
	SENSOR_ATTR_ALERT,
	/** Free-fa duration represented in miiseconds.
	 *  If the sampling frequency is changed during runtime,
	 *  this attribute should be set to adjust freefa duration
	 *  to the new sampling frequency.
	 */
	SENSOR_ATTR_FF_DUR,
	/**
	 * Number of a common sensor attributes.
	 */
	SENSOR_ATTR_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	SENSOR_ATTR_PRIV_START,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	SENSOR_ATTR_MAX,
}

/**
 * @typedef sensor_trigger_handler_t
 * @brief Caback API upon firing of a trigger
 *
 * @param dev Pointer to the sensor device
 * @param trigger The trigger
 */
pub type sensor_trigger_handler_t = fn ( dev :&dyn SensorDevice, trigger :&sensor_trigger);
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[derive(defmt::Format)]
pub enum ErrorKind {
    /// Unspecified error kind.
    Other,
    /// An entity was not found, often a file.
    NotFound,
    /// The operation lacked the necessary privileges to complete.
    PermissionDenied,
    /// The connection was refused by the remote server.
    ConnectionRefused,
    /// The connection was reset by the remote server.
    ConnectionReset,
    /// The connection was aborted (terminated) by the remote server.
    ConnectionAborted,
    /// The network operation failed because it was not connected yet.
    NotConnected,
    /// A socket address could not be bound because the address is already in
    /// use elsewhere.
    AddrInUse,
    /// A nonexistent interface was requested or the requested address was not
    /// local.
    AddrNotAvailable,
    /// The operation failed because a pipe was closed.
    BrokenPipe,
    /// An entity already exists, often a file.
    AlreadyExists,
    /// A parameter was incorrect.
    InvalidInput,
    /// Data not valid for the operation were encountered.
    InvalidData,
    /// The I/O operation's timeout expired, causing it to be canceled.
    TimedOut,
    /// This operation was interrupted.
    Interrupted,
    /// This operation is unsupported on this platform.
    Unsupported,
    /// An operation could not be completed, because it failed
    /// to allocate enough memory.
    OutOfMemory,
    /// An attempted write could not write any data.
    WriteZero,
}
/**
 * @brief Trait supported by Sensor Drivers
 * 
 */
pub trait SensorDevice {

	/**
	 * @brief Check if device is ready
	 */
	 fn device_is_ready (&self) 
	 	-> bool ;

	/**
	 * @brief Initialize Device
	 */
	 fn init (&mut self)					
	 	-> Result<(), ErrorKind> ;
	
	fn name (&self) -> &str;

	/**
	 * @brief Set an attribute for a sensor
	 *
	 * @param dev Pointer to the sensor device
	 * @param chan The channel the attribute belongs to, if any.  Some
	 * attributes may only be set for a channels of a device, depending on
	 * device capabilities.
	 * @param attr The attribute to set
	 * @param val The value to set the attribute to
	 *
	 * @return 0 if successful, negative errno code if failure.
	 */
	 fn sensor_attr_set (&mut self,
					chan :sensor_channel,
					attr :sensor_attribute,
					val : &sensor_value) 
					-> Result<(), ErrorKind> ;

	/**
	 * @brief Get an attribute for a sensor
	 *
	 * @param dev Pointer to the sensor device
	 * @param chan The channel the attribute belongs to, if any.  Some
	 * attributes may only be set for a channels of a device, depending on
	 * device capabilities.
	 * @param attr The attribute to get
	 * @param val Pointer to where to store the attribute
	 *
	 * @return 0 if successful, negative errno code if failure.
	 */

	fn sensor_attr_get ( &self,
					chan :sensor_channel,
					attr :sensor_attribute,
					val : &sensor_value)
					-> Result<(), ErrorKind> ;

	/**
	 * @brief Activate a sensor's trigger and set the trigger handler
	 *
	 * The handler wi be caed from a thread, so I2C or SPI operations are
	 * safe.  However, the thread's stack is limited and defined by the
	 * driver.  It is currently up to the caer to ensure that the handler
	 * does not overflow the stack.
	 *
	 * The user-aocated trigger wi be stored by the driver as a pointer, rather
	 * than a copy, and passed back to the handler. This enables the handler to use
	 * CONTAINER_OF to retrieve a context pointer when the trigger is embedded in a
	 * larger struct and requires that the trigger is not aocated on the stack.
	 *
	 * @funcprops \supervisor
	 *
	 * @param dev Pointer to the sensor device
	 * @param trig The trigger to activate
	 * @param handler The function that should be caed when the trigger
	 * fires
	 *
	 * @return 0 if successful, negative errno code if failure.
	 */
	fn sensor_trigger_set ( &self,
				    trig : &sensor_trigger,
				    handler :sensor_trigger_handler_t)
					-> Result<(), ErrorKind> ;

	/**
	 * @brief Fetch a sample from the sensor and store it in an internal
	 * driver buffer
	 *
	 * Read a of a sensor's active channels and, if necessary, perform any
	 * additional operations necessary to make the values useful.  The user
	 * may then get individual channel values by caing @ref
	 * sensor_channel_get.
	 *
	 * The function blocks until the fetch operation is complete.
	 *
	 * Since the function communicates with the sensor device, it is unsafe
	 * to ca it in an ISR if the device is connected via I2C or SPI.
	 *
	 * @param dev Pointer to the sensor device
	 *
	 * @return 0 if successful, negative errno code if failure.
	 */
	fn sensor_sample_fetch_chan(&mut self,
				     chan :sensor_channel)
					 -> Result<(), ErrorKind> ;
	/**
	 * @brief fetch a sample for all channels. Same as sensor_sample_fetch_chan(SENSOR_CHAN_ALL)
	 */	 
	 fn sensor_sample_fetch(&mut self)
		-> Result<(), ErrorKind> {
		self.sensor_sample_fetch_chan(sensor_channel::SENSOR_CHAN_ALL)
	 }

	/**
	 * @brief Get a reading from a sensor device
	 *
	 * Return a useful value for a particular channel, from the driver's
	 * internal data.  Before caing this function, a sample must be
	 * obtained by caing @ref sensor_sample_fetch or
	 * @ref sensor_sample_fetch_chan. It is guaranteed that two subsequent
	 * cas of this function for the same channels wi yield the same
	 * value, if @ref sensor_sample_fetch or @ref sensor_sample_fetch_chan
	 * has not been caed in the meantime.
	 *
	 * For vectorial data samples you can request a axes in just one ca
	 * by passing the specific channel with _XYZ suffix. The sample wi be
	 * returned at val[0], val[1] and val[2] (X, Y and Z in that order).
	 *
	 * @param dev Pointer to the sensor device
	 * @param chan The channel to read
	 * @param val Where to store the value
	 *
	 * @return 0 if successful, negative errno code if failure.
	 */
	fn sensor_channel_get(&self,
							chan :sensor_channel,
							val :&mut Vec<sensor_value,3>)
							-> Result<(), ErrorKind> ;
}

/**
 * @brief Trait supported by bus drivers for MCU->Sensor communication 
 * (e.g. spi or i2c). This is passed to the sensor driver as a configuration 
 * parameter for sensors that support multiple buses
 */
pub trait SensorBus{
	/**
	 * @brief initialize the bus
	 */
	fn  init(&mut self) 
		-> Result<(), ErrorKind> ;

	/**
	 * @brief check if bus is ready
	 */
	fn check(&self) 
		-> Result<(), ErrorKind> ;

	/**
	 * @brief read data from a sensor register
	 * @param adr address of the register
	 * @param data buffere for receiving data  
	 */
	fn read(&mut self, adr: u8, data: &mut [u8]) 
		-> Result<(), ErrorKind> ;

	/**
	 * @brief write data to a sensor register
	 * @param adr address of the register
	 * @param data buffere for sending data  
	 */
	 fn write(&mut self, adr: u8, data: &[u8]) 
		-> Result<(), ErrorKind>;
}

/**
 * @brief The value of gravitational constant in micro m/s^2.
 */
pub const SENSOR_G :i64 = 9806650;

/**
 * @brief The value of constant PI in micros.
 */
pub const SENSOR_PI :i64 =	3141592;

//TODO: Implement below functions as traits and return Result in case of errors

/**
 * @brief Helper function to convert acceleration from m/s^2 to Gs
 *
 * @param ms2 A pointer to a sensor_value struct holding the acceleration,
 *            in m/s^2.
 *
 * @return The converted value, in Gs.
 */
pub fn sensor_ms2_to_g(ms2 :&sensor_value) ->i32 {
	let micro_ms2: i64 = ms2.val1 as i64* 1000000i64 + ms2.val2 as i64;

	if micro_ms2 > 0 {
		((micro_ms2 + SENSOR_G / 2i64) / SENSOR_G) as i32
	} else {
		((micro_ms2 - SENSOR_G / 2i64) / SENSOR_G) as i32
	}
}

/**
 * @brief Helper function to convert acceleration from Gs to m/s^2
 *
 * @param g The G value to be converted.
 * @param ms2 A pointer to a sensor_value struct, where the result is stored.
 */
pub fn sensor_g_to_ms2(g :i32, ms2 :&mut sensor_value) {
	ms2.val1 = ((g as i64 * SENSOR_G ) / 1000000i64) as i32;
	ms2.val2 = ((g as i64 * SENSOR_G) % 1000000i64) as i32;
}

/**
 * @brief Helper function to convert acceleration from m/s^2 to micro Gs
 *
 * @param ms2 A pointer to a sensor_value struct holding the acceleration,
 *            in m/s^2.
 *
 * @return The converted value, in micro Gs.
 */
pub fn sensor_ms2_to_ug(ms2 :&sensor_value) ->i32 {
	let micro_ms2:i64 = (ms2.val1 as i64* 1000000i64) + ms2.val2 as i64;
	((micro_ms2 as i64* 1000000i64) / SENSOR_G) as i32
}

/**
 * @brief Helper function to convert acceleration from micro Gs to m/s^2
 *
 * @param ug The micro G value to be converted.
 * @param ms2 A pointer to a sensor_value struct, where the result is stored.
 */
pub fn sensor_ug_to_ms2(ug :i32, ms2 :&mut sensor_value) {
	ms2.val1 = ((ug as i64 * SENSOR_G / 1000000i64) / 1000000i64) as i32;
	ms2.val2 = ((ug as i64 * SENSOR_G/ 1000000i64) % 1000000i64) as i32;
}

/**
 * @brief Helper function for converting radians to degrees.
 *
 * @param rad A pointer to a sensor_value struct, holding the value in radians.
 *
 * @return The converted value, in degrees.
 */
pub fn sensor_rad_to_degrees( rad : &sensor_value) ->i32
{
	let micro_rad_s: i64 = rad.val1 as i64 * 1000000i64 + rad.val2 as i64;

	if micro_rad_s > 0 {
		((micro_rad_s * 180i64 + SENSOR_PI / 2i64) / SENSOR_PI) as i32
	} else {
		((micro_rad_s * 180i64 - SENSOR_PI / 2i64) / SENSOR_PI) as i32
	}
}

/**
 * @brief Helper function for converting degrees to radians.
 *
 * @param d The value (in degrees) to be converted.
 * @param rad A pointer to a sensor_value struct, where the result is stored.
 */
pub fn sensor_degrees_to_rad(d: i32, rad :&mut sensor_value)
{
	rad.val1 = ((d as i64 * SENSOR_PI / 180i64) / 1000000i64) as i32 ;
	rad.val2 = ((d as i64 * SENSOR_PI / 180i64) % 1000000i64) as i32;
}

/**
 * @brief Helper function for converting radians to 10 micro degrees.
 *
 * When the unit is 1 micro degree, the range that the int32_t can represent is
 * +/-2147.483 degrees. In order to increase this range, here we use 10 micro
 * degrees as the unit.
 *
 * @param rad A pointer to a sensor_value struct, holding the value in radians.
 *
 * @return The converted value, in 10 micro degrees.
 */
pub fn sensor_rad_to_10udegrees(rad :&sensor_value) ->i32 {
	let micro_rad_s :i64 = rad.val1 as i64 * 1000000i64 + rad.val2 as i64;
	((micro_rad_s * 180i64 * 100000i64) / SENSOR_PI) as i32
}

/**
 * @brief Helper function for converting 10 micro degrees to radians.
 *
 * @param d The value (in 10 micro degrees) to be converted.
 * @param rad A pointer to a sensor_value struct, where the result is stored.
 */
pub fn sensor_10udegrees_to_rad(d: i32, rad :&mut sensor_value)
{
	rad.val1 = ((d as i64 * SENSOR_PI / 180i64 / 100000i64) / 1000000i64) as i32;
	rad.val2 = ((d as i64 * SENSOR_PI / 180i64 / 100000i64) % 1000000i64) as i32;
}

/**
 * @brief Helper function for converting struct sensor_value to double.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
pub fn sensor_value_to_double( val :&sensor_value) ->f64 {
	val.val1 as f64 + val.val2 as f64 / 1000000f64
}

/**
 * @brief Helper function for converting struct sensor_value to float.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
pub fn sensor_value_to_float(val :&sensor_value) ->f32 {
	return val.val1 as f32 + val.val2 as f32 / 1000000f32;
}

/**
 * @brief Helper function for converting double to struct sensor_value.
 *
 * @param val A pointer to a sensor_value struct.
 * @param inp The converted value.
 * @return 0 if successful, negative errno code if failure.
 */
//TODO: Implement below functions as traits and return Result in case of errors

pub fn sensor_value_from_double( val :&mut sensor_value, inp: f64) -> i32 {
	if inp < i32::MIN as f64 || inp > i32::MAX as f64 {
		return -1;
	}
	let inp_i32 = inp as i32;
	let val2 :f64 = (inp - inp_i32 as f64) * 1000000f64;

	if val2 < i32::MIN as f64|| val2 > i32::MAX as f64 {
		return -1;
	}

	val.val1 = inp_i32;
	val.val2 = val2 as i32;

	return 0;
}

/**
 * @brief Helper function for converting float to struct sensor_value.
 *
 * @param val A pointer to a sensor_value struct.
 * @param inp The converted value.
 * @return 0 if successful, negative errno code if failure.
 */

pub fn sensor_value_from_float(val :&mut sensor_value, inp: f32) ->i32
{
	let inp_i32 = inp as i32;
	let val2 :f32 = (inp - inp_i32 as f32) * 1000000f32;

	if val2 < i32::MIN as f32 || val2 > (i32::MAX - 1) as f32{
		return -1;
	}

	val.val1 = inp_i32;
	val.val2 = val2 as i32;

	return 0;
}

/**
 * @brief Helper function for converting struct sensor_value to integer mii units.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
pub fn sensor_value_to_mii( val :&sensor_value) ->i64 {
	return (val.val1 as i64 * 1000i64) + val.val2 as i64/ 1000i64;
}

/**
 * @brief Helper function for converting struct sensor_value to integer micro units.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
pub fn sensor_value_to_micro( val :&sensor_value) -> i64 {
	(val.val1 as i64* 1000000i64) + val.val2 as i64
}

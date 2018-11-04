#[macro_use]
extern crate enum_primitive_derive;
extern crate num_traits;
use num_traits::FromPrimitive;

#[link(name = "vl53l1x_api")]
extern "C" {
    fn initI2c(i2c_bus: u8, i2c_addr: u8) -> u8;
    fn initSensor(device_id: u8) -> u8;
    fn softwareReset(device_id: u8) -> u8;
    fn release(device_id: u8);
    fn startRanging(device_id: u8, mode: u8) -> u8;
    fn getRangingMeasurement(device_id: u8) -> RangingMeasurement;
    fn stopRanging(device_id: u8) -> u8;
    fn setDeviceAddress(device_id: u8, i2c_addr: u8) -> u8;
    fn getUserROI(device_id: u8) -> UserRoi;
    fn setUserROI(
        device_id: u8,
        top_left_x: u8,
        top_left_y: u8,
        bot_right_x: u8,
        bot_right_y: u8,
    ) -> u8;
    fn getMeasurementTimingBudgetMicroSeconds(device_id: u8) -> u32;
    fn setMeasurementTimingBudgetMicroSeconds(device_id: u8, timing_budget_us: u32) -> u8;
    fn getInterMeasurementPeriodMilliSeconds(device_id: u8) -> u32;
    fn setInterMeasurementPeriodMilliSeconds(device_id: u8, inter_measurement_period: u32) -> u8;
}

#[derive(Debug)]
#[repr(C)]
struct RangingMeasurement {
    timestamp: u32,
    stream_count: u8,
    range_quality_level: u8,
    signal_rate_rtn_mega_cps: u32,
    ambient_rate_rtn_mega_cps: u32,
    effective_spad_rtn_count: u16,
    sigma_milli_meter: u32,
    range_milli_meter: i16,
    range_fractional_part: u8,
    range_status: u8,
}

#[derive(Debug)]
#[repr(C)]
pub struct UserRoi {
    top_left_x: u8,
    top_left_y: u8,
    bot_right_x: u8,
    bot_right_y: u8,
}

/// Check `status` to decide whether the `distance` field is valid.
#[derive(Debug)]
pub struct Vl53l1xSample {
    /// Range distance in mm.
    /// Empirically, if distance is 0, there's something wrong even though the
    /// range status is Ok.
    pub distance: u16,
    /// Measure of target reflectance.
    pub signal_rate: u32,
    /// Measure of ambient light.
    pub ambient_rate: u32,
    pub spad_count: u16,
    pub status: Vl53l1xRangeStatus,
}

#[derive(Debug)]
pub enum Vl53l1xReadSampleError {
    /// Reported range was negative.
    BadRange(i16),
    /// Range status was not decodable to documented status.
    BadRangeStatus(u8),
}

/// The VL53L1X Time of Flight sensor.
pub struct Vl53l1x {
    i2c_addr: u8,
    i2c_dev: u8,
}

impl Vl53l1x {
    /// Connects to VL53L1X.
    ///
    /// If i2c_addr is None, defaults to 0x29.
    pub fn new(i2c_bus: i32, i2c_addr: Option<u8>) -> Result<Self, Vl53l1xError> {
        let i2c_addr = i2c_addr.unwrap_or(0x29);
        let i2c_dev = unsafe { initI2c(i2c_bus as u8, i2c_addr) };
        if i2c_dev > 100 {
            return Err(Vl53l1xError::from_u8(i2c_dev).unwrap());
        }
        Ok(Self { i2c_addr, i2c_dev })
    }

    pub fn soft_reset(&mut self) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = softwareReset(self.i2c_dev);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    pub fn init(&mut self) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = initSensor(self.i2c_dev);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    /// Defaults timing budget to 66ms and inter measurement period to 70ms.
    /// To change from defaults, call corresponding functions after this.
    pub fn start_ranging(&mut self, mode: DistanceMode) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = startRanging(self.i2c_dev, mode as u8);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    pub fn stop_ranging(&mut self) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = stopRanging(self.i2c_dev);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    pub fn read_sample(&mut self) -> Result<Vl53l1xSample, Vl53l1xReadSampleError> {
        unsafe {
            let m = getRangingMeasurement(self.i2c_dev);
            if m.range_milli_meter < 0 {
                return Err(Vl53l1xReadSampleError::BadRange(m.range_milli_meter));
            }
            let range_status = Vl53l1xRangeStatus::from_u8(m.range_status);
            if range_status.is_none() {
                // While all documented statuses are enumerated, empirically,
                // have seen other values.
                return Err(Vl53l1xReadSampleError::BadRangeStatus(m.range_status));
            }
            Ok(Vl53l1xSample {
                distance: m.range_milli_meter as u16,
                signal_rate: m.signal_rate_rtn_mega_cps,
                ambient_rate: m.ambient_rate_rtn_mega_cps,
                spad_count: m.effective_spad_rtn_count,
                // Should be safe to unwrap as all *documented* statuses are
                // enumerated.
                status: range_status.unwrap(),
            })
        }
    }

    /// Change the i2c slave address of the device.
    pub fn set_device_address(&mut self, new_i2c_addr: u8) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = setDeviceAddress(self.i2c_dev, new_i2c_addr);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        self.i2c_addr = new_i2c_addr;
        Ok(())
    }
    pub fn get_user_roi(&mut self) -> UserRoi {
        unsafe { getUserROI(self.i2c_dev) }
    }

    /// Set region of interest. Minimum width and height is 3.
    pub fn set_user_roi(
        &mut self,
        top_left_x: u8,
        top_left_y: u8,
        bot_right_x: u8,
        bot_right_y: u8,
    ) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = setUserROI(
                self.i2c_dev,
                top_left_x,
                top_left_y,
                bot_right_x,
                bot_right_y,
            );
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    /// Return value in microseconds.
    pub fn get_measurement_timing_budget(&mut self) -> u32 {
        unsafe { getMeasurementTimingBudgetMicroSeconds(self.i2c_dev) }
    }

    /// 20ms is the minimum timing budget for short distance mode.
    /// 33ms is the minimum timing budget that works for all distance modes.
    /// 140ms is the timing budget needed to measure up to the maximum distance
    /// of 4 meters in ideal conditions.
    /// `timing_budget` units are microseconds.
    pub fn set_measurement_timing_budget(
        &mut self,
        timing_budget: u32,
    ) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = setMeasurementTimingBudgetMicroSeconds(self.i2c_dev, timing_budget);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }

    /// Return value in milliseconds.
    pub fn get_inter_measurement_period(&mut self) -> u32 {
        unsafe { getInterMeasurementPeriodMilliSeconds(self.i2c_dev) }
    }

    /// Inter measurement period must be greater than timing budget.
    /// `inter_measurement_period` units are milliseconds.
    pub fn set_inter_measurement_period(
        &mut self,
        inter_measurement_period: u32,
    ) -> Result<(), Vl53l1xError> {
        unsafe {
            let res = setInterMeasurementPeriodMilliSeconds(self.i2c_dev, inter_measurement_period);
            if res != 0 {
                return Err(Vl53l1xError::from_u8(res).unwrap());
            }
        }
        Ok(())
    }
}

impl Drop for Vl53l1x {
    fn drop(&mut self) {
        unsafe {
            release(self.i2c_dev);
        }
    }
}

pub enum DistanceMode {
    /// Max distance: 1360mm (dark), 1350mm (ambient)
    Short = 1,
    /// Max distance: 2900mm (dark), 760mm (ambient)
    Mid = 2,
    /// Max distance: 3600mm (dark), 730mm (ambient)
    Long = 3,
}

#[derive(Debug, Primitive)]
pub enum Vl53l1xRangeStatus {
    Ok = 0,
    SigmaFail = 1,
    /// Empirically observed when distance is out of bounds.
    SignalFail = 2,
    RangeValidMinRangeClipped = 3,
    /// Empirically observed when distance is out of bounds.
    OutOfboundsFail = 4,
    HardwareFail = 5,
    RangeValidNoWrapCheckFail = 6,
    /// Empirically observed when distance changes radically (put hand in front
    /// of sensor), and when distance is out of bounds.
    WrapTargetFail = 7,
    ProcessingFail = 8,
    XtalkSignalFail = 9,
    SyncronisationInt = 10,
    RangeValidMergedPulse = 11,
    TargetPresentLackOfSignal = 12,
    MinRangeFail = 13,
    RangeInvalid = 14,
}

#[derive(Debug, Primitive)]
pub enum Vl53l1xError {
    CalibrationWarning = 255,
    MinClipped = 254,
    Undefined = 253,
    InvalidParams = 252,
    NotSupported = 251,
    RangeError = 250,
    TimeOut = 249,
    ModeNotSupported = 248,
    BufferTooSmall = 247,
    CommsBufferTooSmall = 246,
    GpioNotExisting = 245,
    GpioFunctionalityNotSupported = 244,
    ControlInterface = 243,
    InvalidCommand = 242,
    DivisionByZero = 241,
    RefSpadInit = 240,
    GphSyncCheckFail = 239,
    StreamCountCheckFail = 238,
    GphIdCheckFail = 237,
    ZoneStreamCountCheckFail = 236,
    ZoneGphIdCheckFail = 235,
    XtalkExtractionNoSampleFail = 234,
    XtalkExtractionSigmaLimitFail = 233,
    OffsetCalNoSampleFail = 232,
    OffsetCalNoSpadsEnabledFail = 231,
    ZoneCalNoSampleFail = 230,
    TuningParmKeyMismatch = 229,
    WarningRefSpadCharNotEnoughSpads = 228,
    WarningRefSpadCharRateTooHigh = 227,
    WarningRefSpadCharRateTooLow = 226,
    WarningOffsetCalMissingSamples = 225,
    WarningOffsetCalSigmaTooHigh = 224,
    WarningOffsetCalRateTooHigh = 223,
    WarningOffsetCalSpadCountTooLow = 222,
    WarningZoneCalMissingSamples = 221,
    WarningZoneCalSigmaTooHigh = 220,
    WarningZoneCalRateTooHigh = 219,
    WarningXtalkMissingSamples = 218,
    WarningXtalkNoSamplesForGradient = 217,
    WarningXtalkSigmaLimitForGradient = 216,
    NotImplemented = 215,
    PlatformSpecificStart = 196,
}

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

    pub fn read_sample(&mut self) -> Vl53l1xSample {
        unsafe {
            let m = getRangingMeasurement(self.i2c_dev);
            assert!(m.range_milli_meter >= 0);
            Vl53l1xSample {
                distance: m.range_milli_meter as u16,
                signal_rate: m.signal_rate_rtn_mega_cps,
                ambient_rate: m.ambient_rate_rtn_mega_cps,
                spad_count: m.effective_spad_rtn_count,
                // Should be safe to unwrap as all *documented* statuses are
                // enumerated.
                status: Vl53l1xRangeStatus::from_u8(m.range_status).unwrap(),
            }
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

extern crate byteorder;
use byteorder::{BigEndian, ByteOrder};
extern crate i2cdev;
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use std::thread;
use std::time;

pub trait I2cInterface {
    fn addr(&self) -> u16;
}

pub trait Register {
    fn mask(&self) -> u8;
}

pub fn get_i2c_bus_path(i2c_bus: i32) -> String {
    format!("/dev/i2c-{}", i2c_bus)
}

pub const VL53L1_I2C_ADDR: u16 = 0x29;

#[allow(dead_code)]
enum Vl53l1xReg {
    IdentificationModelId,
    SoftReset,
    FirmwareSystemStatus,
    PadI2cHvExtsupConfig,
    GpioTioHvStatus,
    ResultFinalCrosstalkCorrectedRangeMmSd0,
    ResultPeakSignalCountRateCrosstalkCorrectedMcpsSd0,
    RangeConfigVcselPeriodA,
    RangeConfigVcselPeriodB,
    RangeConfigValidPhaseHigh,
    SdConfigWoiSd0,
    SdConfigWoiSd1,
    SdConfigInitialPhaseSd0,
    SdConfigInitialPhaseSd1,
}

impl I2cInterface for Vl53l1xReg {
    fn addr(&self) -> u16 {
        match *self {
            Vl53l1xReg::IdentificationModelId => 0x010f,
            Vl53l1xReg::SoftReset => 0x0000,
            Vl53l1xReg::FirmwareSystemStatus => 0x00e5,
            Vl53l1xReg::PadI2cHvExtsupConfig => 0x002e,
            Vl53l1xReg::GpioTioHvStatus => 0x0031,
            Vl53l1xReg::ResultFinalCrosstalkCorrectedRangeMmSd0 => 0x0096,
            Vl53l1xReg::ResultPeakSignalCountRateCrosstalkCorrectedMcpsSd0 => 0x0098,
            Vl53l1xReg::RangeConfigVcselPeriodA => 0x0060,
            Vl53l1xReg::RangeConfigVcselPeriodB => 0x0063,
            Vl53l1xReg::RangeConfigValidPhaseHigh => 0x0069,
            Vl53l1xReg::SdConfigWoiSd0 => 0x0078,
            Vl53l1xReg::SdConfigWoiSd1 => 0x0079,
            Vl53l1xReg::SdConfigInitialPhaseSd0 => 0x007a,
            Vl53l1xReg::SdConfigInitialPhaseSd1 => 0x007b,
        }
    }
}

pub struct Vl53l1x {
    i2c_dev: LinuxI2CDevice,
    config: Vec<u8>,
}

const MODEL_ID: u16 = 0xeacc;

impl Vl53l1x {

    /// Connects to VL53L1X.
    ///
    /// If i2c_addr is None, defaults to 0x29.
    pub fn new(i2c_bus: i32, i2c_addr: Option<u16>)
               -> Result<Vl53l1x, LinuxI2CError> {
        let i2c_dev = LinuxI2CDevice::new(
            get_i2c_bus_path(i2c_bus), i2c_addr.unwrap_or(0x29))?;

        let mut vl = Vl53l1x {
            i2c_dev,
            config: CONFIG.to_vec(),
        };

        vl.check_model_id()?;
        vl.soft_reset()?;
        vl.wait_for_firmware_status_ready()?;
        vl.write_i2c_28v()?;
        vl.get_trim_resistors()?;

        return Ok(vl);
    }

    fn check_model_id(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(
            &addr_to_bytes(Vl53l1xReg::IdentificationModelId.addr()))?;

        let mut buf2 = [0u8, 2];
        self.i2c_dev.read(&mut buf2)?;
        let test_model_id = BigEndian::read_u16(&buf2);

        if test_model_id != MODEL_ID {
            panic!("Unexpected model_id: {} != {}", test_model_id, MODEL_ID);
        }

        Ok(())
    }

    fn soft_reset(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(&[
            (Vl53l1xReg::SoftReset.addr() >> 8) as u8,
            (Vl53l1xReg::SoftReset.addr() & 0xff) as u8,
            0x00,
        ])?;

        thread::sleep(time::Duration::from_micros(100));

        self.i2c_dev.write(&[
            (Vl53l1xReg::SoftReset.addr() >> 8) as u8,
            (Vl53l1xReg::SoftReset.addr() & 0xff) as u8,
            0x01,
        ])?;

        thread::sleep(time::Duration::from_micros(200));

        Ok(())
    }

    fn wait_for_firmware_status_ready(&mut self) -> Result<(), LinuxI2CError> {
        let mut attempts = 0;
        let mut buf2 = [0u8, 2];
        loop {
            self.i2c_dev.write(
            &addr_to_bytes(Vl53l1xReg::FirmwareSystemStatus.addr()))?;
                self.i2c_dev.read(&mut buf2)?;
            let system_status = BigEndian::read_u16(&buf2);
            if system_status & 0x0001 != 0 {
                attempts += 1;
                if attempts > 100 {
                    // TODO: Change to something recoverable.
                    panic!("Sensor timed out.")
                }
                thread::sleep(time::Duration::from_millis(10));
            } else {
                break;
            }
        }

        Ok(())
    }

    fn write_i2c_28v(&mut self) -> Result<(), LinuxI2CError> {
        let mut reg_value = i2c_read_u16(
            &mut self.i2c_dev, Vl53l1xReg::PadI2cHvExtsupConfig.addr()).unwrap();
        reg_value = (reg_value & 0xfe) | 0x01;

        i2c_write_u16(
            &mut self.i2c_dev, Vl53l1xReg::PadI2cHvExtsupConfig.addr(), reg_value)?;

        Ok(())
    }

    fn get_trim_resistors(&mut self) -> Result<(), LinuxI2CError> {
        for i in 0..36 {
            self.config[i] = i2c_read_u8(&mut self.i2c_dev, (i + 1) as u16).unwrap();
        }
        Ok(())
    }

    pub fn start_measurement(&mut self) -> Result<(), LinuxI2CError> {
        let mut config = self.config.clone();
        // Add address (0x0001)
        config.insert(0, 0x01);
        config.insert(0, 0x00);
        self.i2c_dev.write(&config)?;
        Ok(())
    }

    pub fn check_data_ready(&mut self) -> Result<bool, LinuxI2CError> {
        let val = i2c_read_u8(
            &mut self.i2c_dev, Vl53l1xReg::GpioTioHvStatus.addr()).unwrap();
        Ok(val != 0x03)
    }

    pub fn wait_data_ready(&mut self) -> Result<(), LinuxI2CError> {
        while !self.check_data_ready()? {
            thread::sleep(time::Duration::from_millis(5));
        }
        Ok(())
    }

    pub fn read_distance(&mut self) -> Result<u16, LinuxI2CError> {
        let val = i2c_read_u16(
            &mut self.i2c_dev,
            Vl53l1xReg::ResultFinalCrosstalkCorrectedRangeMmSd0.addr()).unwrap();
        Ok(val)
    }

    pub fn read_signal_rate(&mut self) -> Result<u16, LinuxI2CError> {
        let val = i2c_read_u16(
            &mut self.i2c_dev,
            Vl53l1xReg::ResultPeakSignalCountRateCrosstalkCorrectedMcpsSd0.addr()).unwrap();
        Ok(val)
    }

    pub fn read_measurement(&mut self) -> Result<Measurement, LinuxI2CError> {
        self.i2c_dev.write(
            &addr_to_bytes(Vl53l1xReg::ResultFinalCrosstalkCorrectedRangeMmSd0.addr()))?;
        let mut buf4 = [0u8; 6];
        self.i2c_dev.read(&mut buf4)?;

        Ok(Measurement {
            distance: BigEndian::read_u16(&buf4[0 .. 2]),
            signal_rate: BigEndian::read_u16(&buf4[2 .. 4]),
        })
    }

    pub fn write_distance_mode(&mut self, mode: DistanceMode) -> Result<(), LinuxI2CError> {
        let period_a;
        let period_b;
        let phase_high;
        let phase_init;

        match mode {
            DistanceMode::Short => {
                period_a = 0x07;
                period_b = 0x05;
                phase_high = 0x38;
                phase_init = 6;
            },
            DistanceMode::Mid => {
                period_a = 0x0f;
                period_b = 0x0d;
                phase_high = 0xb8;
                phase_init = 14;
            },
            DistanceMode::Long => {
                period_a = 0x0f;
                period_b = 0x0d;
                phase_high = 0xb8;
                phase_init = 14;
            }
        }

        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::RangeConfigVcselPeriodA.addr(), period_a)?;
        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::RangeConfigVcselPeriodB.addr(), period_b)?;
        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::RangeConfigValidPhaseHigh.addr(), phase_high)?;

        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::SdConfigWoiSd0.addr(), period_a)?;
        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::SdConfigWoiSd1.addr(), period_b)?;
        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::SdConfigInitialPhaseSd0.addr(), phase_init)?;
        i2c_write_u8(
            &mut self.i2c_dev, Vl53l1xReg::SdConfigInitialPhaseSd1.addr(), phase_init)?;

        Ok(())
    }
}

pub enum DistanceMode {
    Short,
    Mid,
    Long,
}

#[derive(Debug)]
pub struct Measurement {
    pub distance: u16,
    pub signal_rate: u16,
}

fn addr_to_bytes(addr: u16) -> [u8; 2] {
    [
        (addr >> 8) as u8,
        (addr & 0xff) as u8,
    ]
}

fn i2c_read_u8(i2c_dev: &mut LinuxI2CDevice, addr: u16) -> Result<u8, LinuxI2CError> {
    i2c_dev.write(
        &addr_to_bytes(addr))?;
    let mut buf1 = [0u8, 1];
    i2c_dev.read(&mut buf1)?;

    Ok(buf1[0])
}

fn i2c_read_u16(i2c_dev: &mut LinuxI2CDevice, addr: u16) -> Result<u16, LinuxI2CError> {
    i2c_dev.write(
        &addr_to_bytes(addr))?;
    let mut buf2 = [0u8, 2];
    i2c_dev.read(&mut buf2)?;

    Ok(BigEndian::read_u16(&buf2))
}

fn i2c_write_u8(i2c_dev: &mut LinuxI2CDevice, addr: u16, val: u8) -> Result<(), LinuxI2CError> {
    i2c_dev.write(&[
        (addr >> 8) as u8,
        (addr & 0xff) as u8,
        val,
    ])?;

    Ok(())
}

fn i2c_write_u16(i2c_dev: &mut LinuxI2CDevice, addr: u16, val: u16) -> Result<(), LinuxI2CError> {
    i2c_dev.write(&[
        (addr >> 8) as u8,
        (addr & 0xff) as u8,
        (val >> 8) as u8,
        (val & 0xff) as u8,
    ])?;

    Ok(())
}

const CONFIG: [u8; 135] = [
  0x29, 0x02, 0x10, 0x00, 0x28, 0xBC, 0x7A, 0x81,
  0x80, 0x07, 0x95, 0x00, 0xED, 0xFF, 0xF7, 0xFD,
  0x9E, 0x0E, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00,
  0x28, 0x00, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
  0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
  0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00,
  0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x02, 0x00,
  0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x0F,
  0x00, 0xA5, 0x0D, 0x00, 0x80, 0x00, 0x0C, 0x08,
  0xB8, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F,
  0x0D, 0x0E, 0x0E, 0x01, 0x00, 0x02, 0xC7, 0xFF,
  0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40,
];

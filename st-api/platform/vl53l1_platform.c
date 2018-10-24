#include "vl53l1_platform.h"
#include "vl53l1_api.h"

#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

int num_devices = 0;
// Maximum of 20 device connections.
VL53L1_DEV devices[20];

/*
 * Functions exposed to Rust wrapper.
 */

uint8_t initI2c(uint8_t bus, uint8_t i2c_address)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_Dev_t *dev = (VL53L1_Dev_t *)malloc(sizeof(VL53L1_Dev_t));
    memset(dev, 0, sizeof(VL53L1_Dev_t));

    char device_path[100];
    sprintf(device_path, "/dev/i2c-%d", bus);
    int i2c_fd = open(device_path, O_RDWR);
    ioctl(i2c_fd, I2C_SLAVE, i2c_address);

    dev->I2cDevAddr = i2c_address;
    dev->i2c_fd = i2c_fd;
    uint8_t device_id = num_devices++;
    devices[device_id] = dev;
    return device_id + 1;
}

VL53L1_Error softwareReset(uint8_t device_id) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    VL53L1_DEV dev = devices[device_id - 1];
    Status = VL53L1_software_reset(dev);
    if (Status != VL53L1_ERROR_NONE) { return Status; }
    return VL53L1_WaitDeviceBooted(dev);
}

VL53L1_Error initSensor(uint8_t device_id) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    VL53L1_DEV dev = devices[device_id - 1];
    Status = VL53L1_DataInit(dev);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    Status = VL53L1_StaticInit(dev);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    VL53L1_DeviceInfo_t DeviceInfo;
    Status = VL53L1_GetDeviceInfo(dev, &DeviceInfo);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    Status = VL53L1_PerformRefSpadManagement(dev);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    return VL53L1_SetXTalkCompensationEnable(dev, 0);
}

void release(uint8_t device_id) {
    VL53L1_DEV dev = devices[device_id - 1];
    free(dev);
}

VL53L1_Error startRanging(uint8_t device_id, VL53L1_DistanceModes mode)
{
    VL53L1_DEV dev = devices[device_id - 1];
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    Status = VL53L1_SetDistanceMode(dev, mode);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 66000);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 70);
    if (Status != VL53L1_ERROR_NONE) { return Status; }

    return VL53L1_StartMeasurement(dev);
}

VL53L1_RangingMeasurementData_t getRangingMeasurement(uint8_t device_id)
{
    VL53L1_DEV dev = devices[device_id - 1];
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_RangingMeasurementData_t RangingMeasurement;

    // FIXME: Handle status
    Status = VL53L1_WaitMeasurementDataReady(dev);
    Status = VL53L1_GetRangingMeasurementData(dev, &RangingMeasurement);
    Status = VL53L1_ClearInterruptAndStartMeasurement(dev);
    return RangingMeasurement;
}

VL53L1_Error stopRanging(uint8_t device_id)
{
    VL53L1_DEV dev = devices[device_id - 1];
    return VL53L1_StopMeasurement(dev);
}

VL53L1_Error setDeviceAddress(uint8_t device_id, uint8_t i2c_address) {
    VL53L1_DEV dev = devices[device_id - 1];
    VL53L1_Error Status = VL53L1_SetDeviceAddress(dev, i2c_address << 1);
    ioctl(dev->i2c_fd, I2C_SLAVE, i2c_address);
    dev->I2cDevAddr = i2c_address;
    return Status;
}

VL53L1_UserRoi_t getUserROI(uint8_t device_id) {
    VL53L1_DEV dev = devices[device_id - 1];
    VL53L1_UserRoi_t roi;
    VL53L1_GetUserROI(dev, &roi);
    return roi;
}

VL53L1_Error setUserROI(uint8_t device_id, uint8_t top_left_x, uint8_t top_left_y,
        uint8_t bot_right_x, uint8_t bot_right_y) {
    VL53L1_DEV dev = devices[device_id - 1];
    VL53L1_UserRoi_t roi;
    roi.TopLeftX = top_left_x;
    roi.TopLeftY = top_left_y;
    roi.BotRightX = bot_right_x;
    roi.BotRightY = bot_right_y;
    return VL53L1_SetUserROI(dev, &roi);
}

/*
 * Linux-i2c Implementation
 */

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    char data[200];
    data[0] = index >> 8;
    data[1] = index & 0xff;
    for (int i=0; i<count; i++) {
        data[i+2] = pdata[i];
    }
    write(Dev->i2c_fd, data, 2 + count);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t addr[2];
    addr[0] = index >> 8;
    addr[1] = index & 0xff;
    write(Dev->i2c_fd, addr, 2);
    read(Dev->i2c_fd, pdata, count);

    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    VL53L1_WriteMulti(Dev, index, &data, 1);

    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    char buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xff;
    VL53L1_WriteMulti(Dev, index, buf, 2);

    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    char buf[4];
    buf[0] = data >> 24;
    buf[1] = (data >> 16) & 0xff;
    buf[2] = (data >> 8) & 0xff;
    buf[3] = data & 0xff;
    VL53L1_WriteMulti(Dev, index, buf, 4);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t data;
    VL53L1_RdByte(Dev, index, &data);
    data = ((data & AndData) | OrData);
    VL53L1_WrByte(Dev, index, data);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    VL53L1_ReadMulti(Dev, index, data, 1);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    VL53L1_ReadMulti(Dev, index, buf, 2);
    *data = (buf[0] << 8) | buf[1];
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    VL53L1_ReadMulti(Dev, index, (uint8_t *)data, 4);
    *data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

// Not implemented yet.
VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

// Not implemented yet.
VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	VL53L1_WaitUs(pdev, wait_ms * 1000);
	VL53L1_Error status = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
    usleep(wait_us);
	VL53L1_Error status = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	uint32_t attempts = timeout_ms / poll_delay_ms;
	for (uint32_t x = 0; x < attempts; x++) {
	    uint8_t reg_val = 0;
		VL53L1_Error status = VL53L1_RdByte(pdev, index, &reg_val);
		if (status != VL53L1_ERROR_NONE) {
		    return status;
		} else if ((reg_val & mask) == value) {
			return VL53L1_ERROR_NONE;
		}
		usleep(poll_delay_ms * 1000);
	}
	return VL53L1_ERROR_TIME_OUT;
}

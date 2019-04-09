#include "legato.h"
#include "interfaces.h"
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "i2c-utils.h"


#define I2C_HUB_PORT_RASPI		0x08
#define I2C_HUB_PORT_IOT		0x01
#define I2C_HUB_PORT_GPIO		0x04
#define I2C_HUB_PORT_USB_HUB		0x02
#define I2C_HUB_PORT_ALL		0x0F
#define DEFAULT_I2C_ADDR		0x40


char laser_sensor_i2c_bus[256] = "/dev/i2c-0";
uint8_t buf[29]={0,};
size_t buf_len = sizeof(buf);
/**
 * Function: Configure I2C hub to enable I2C bus that connected to led matrix
 * Params:  - hub_address: I2C address of hub
 *      - port: Bus ports
 **/
int i2c_hub_select_port(uint8_t hub_address, uint8_t port)
{
	int result;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, hub_address) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			hub_address,strerror(errno));
		return 1;
	}
	int writeResult = i2c_smbus_write_byte(i2c_fd, port);
	if (writeResult < 0) {
		LE_ERROR("smbus write failed with error %d\n", writeResult);
		result = 1;
	} else {
		result = 0;
	}
	close(i2c_fd);
	return result;
}

/**@brief I2C write byte
 * @param reg :Register address of operation object
 * @param byte :The byte to be wrote.
 * @return result of operation,non-zero if failed.
 * */
int I2C_write_byte(uint8_t reg, uint8_t byte)
{
	int res;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
	LE_ERROR("i2cSendByte: failed to open %s\n", laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
		DEFAULT_I2C_ADDR, strerror(errno));
		return 1;
	}
	res = i2c_smbus_write_byte_data(i2c_fd, reg, byte);
	close(i2c_fd);
	if (!res)
		return 0;
	else
		return 1;
}
/**@brief I2C write 16bit value
 * @param reg: Register address of operation object
 * @param value: The 16bit value to be wrote .
 * @return result of operation,non-zero if failed.
 * */
int I2C_write_16bit(uint8_t reg, uint16_t value)
{
	int res = 0;
	uint8_t val[2] = {0,};
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			DEFAULT_I2C_ADDR, strerror(errno));
		return 1; 
	}
	val[0] = 0xff&(value>>8);
	val[1] = value&0xff;
	res = i2c_smbus_write_i2c_block_data(i2c_fd, reg, 2, val);
	close(i2c_fd);
	if (!res)
		return 0;
	else
		return 1;
}

/**@brief I2C read byte
 * @param reg: Register address of operation object
 * @param byte: The byte to be read in.
 * @return result of operation,non-zero if failed.
 * */
int I2C_read_byte(uint8_t reg, uint8_t* byte)
{
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n",
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			DEFAULT_I2C_ADDR, strerror(errno));
		return 1;
	}
	*byte = i2c_smbus_read_byte_data(i2c_fd, reg);
	close(i2c_fd);
	return 0;
}


/**@brief I2C read 16bit value
 * @param reg: Register address of operation object
 * @param byte: The 16bit value to be read in.
 * @return result of operation,non-zero if failed.
 * */
int I2C_read_16bit(uint8_t start_reg, uint16_t *value)
{	
	*value = 0;
	uint8_t tmp[2] = {0,};
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			DEFAULT_I2C_ADDR, strerror(errno));
		return 1;
	}
	i2c_smbus_read_i2c_block_data(i2c_fd, start_reg, 2, tmp);
	*value|=(uint16_t)tmp[0]<<8;
	*value|=tmp[1];
	close(i2c_fd);   
	return 0;
}

/**@brief I2C read some bytes
 * @param reg: Register address of operation object
 * @param data: The buf  to be read in.
 * @param data_len: The length of buf need to read in.
 * @return result of operation,non-zero if failed.
 * */
int I2C_read_bytes(uint8_t start_reg, uint8_t *data, uint32_t data_len)
{
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			DEFAULT_I2C_ADDR, strerror(errno));
		return 1;
	}
	i2c_smbus_read_i2c_block_data(i2c_fd, start_reg, data_len, data);
	close(i2c_fd);
	return 0;
}

int I2C_read_sensor_value(uint8_t *data, uint32_t data_len)
{	
	int ret = 0;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s \n",
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_I2C_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			DEFAULT_I2C_ADDR, strerror(errno));
		close(i2c_fd);
		return 1;
	}
	i2c_smbus_read_i2c_block_data(i2c_fd, 0, data_len, data);
	close(i2c_fd);
	return ret;
}

void ma_airSensor_Init()
{
	i2c_hub_select_port(0x71, I2C_HUB_PORT_ALL);
}

le_result_t ma_airSensor_ReadIndustrialPM1_0(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (buf_len != 29)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++)
	{
		if (i == 2) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

le_result_t ma_airSensor_ReadIndustrialPM2_5(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++)
	{
		if (i == 3) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

le_result_t ma_airSensor_ReadIndustrialPM10(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++)
	{
		if (i == 4) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

le_result_t ma_airSensor_ReadEnvironmentPM1_0(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++)
	{
		if (i == 5) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

le_result_t ma_airSensor_ReadEnvironmentPM2_5(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++) {
		if (i == 6) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

le_result_t ma_airSensor_ReadEnvironmentPM10(uint16_t *value)
{
	*value = 0;
	int res = 0;
	res = I2C_read_sensor_value(buf, buf_len);
	if (res != 0)
		return LE_FAULT;
	if (NULL == buf)
		return LE_FAULT;
	for (int i = 1; i < 8; i++) {
		if (i == 7) {
			*value = (uint16_t)buf[i*2]<<8|(buf[i*2+1]);
		}
	}
	sleep(5);
	return LE_OK;
}

COMPONENT_INIT
{
	LE_INFO("Air sensor start!");
}

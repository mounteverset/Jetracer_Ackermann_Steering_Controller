// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

class I2C_Driver
{	
	/**
	 * @brief Describe the State the i2c_driver is in
	 * closed = 0
	 * open = 1
	 * 
	 */
    enum class I2C_State : int
	{
		closed = 0,
		open = 1,
	};

public:
	/**
	 * @brief Construct a new i2c driver object
	 * 
	 * @param device_name 
	 */
	I2C_Driver(const char * device_name);

    // Getter and Setter Func
	/**
	 * @brief Get the device name object
	 * 
	 * @return const char* 
	 */
    const char * get_device_name();
	/**
	 * @brief Get the state object
	 * 
	 * @return int 
	 */
	int get_state();
	/**
	 * @brief Get the file descriptor object
	 * 
	 * @return int 
	 */
	int get_file_descriptor();

    // i2c functions
	/**
	 * @brief Open the communication with an i2c device
	 * 
	 * @return true 
	 * @return false 
	 */
    bool open_i2c_device();

	/**
	 * @brief Close the Communication with an i2c device
	 * 
	 * @return true 
	 * @return false 
	 */
	bool close_i2c_device();

	/**
	 * @brief Write data to the I2C device
	 * 
	 * @param address Adress of the I2C device as hex 
	 * @param num_write_btyes 
	 * @param write_data_array 
	 * @return true 
	 * @return false 
	 */
	bool write_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array);

	/**
	 * @brief Write data to the I2C device and directly read data
	 * 
	 * @param address 
	 * @param num_write_btyes 
	 * @param write_data_array 
	 * @param num_read_btyes 
	 * @param read_data_array 
	 * @return true 
	 * @return false 
	 */
	bool write_data_then_read_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array, uint16_t num_read_btyes, uint8_t * read_data_array);


private:

	/**
	 * @brief Names can not be longer than 20 chars
	 * 
	 */
	static const int MAX_DEVICE_NAME_LENGTH = 20;

	/**
	 * @brief Device name
	 * 
	 */
	char m_device_name[20];

	/**
	 * @brief Stores the I2C_Driver state as Enum
	 * 
	 */
	I2C_Driver::I2C_State m_state;

	/**
	 * @brief Store the file descriptor of the I2C Driver
	 * 
	 */
	int m_file_descriptor;

};

#endif // I2C_DRIVER_H
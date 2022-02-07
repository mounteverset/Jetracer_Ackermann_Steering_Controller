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
    enum class I2C_State : int
	{
		closed = 0,
		open = 1,
	};

public:
	I2C_Driver(const char * device_name);

    // Getter and Setter Func
    const char * get_device_name();
	int get_state();
	int get_file_descriptor();


    // i2c functions
    bool open_i2c_device();
	bool close_i2c_device();

	bool write_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array);
	bool write_data_then_read_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array, uint16_t num_read_btyes, uint8_t * read_data_array);


private:
	static const int MAX_DEVICE_NAME_LENGTH = 20;
	char m_device_name[20];
	I2C_Driver::I2C_State m_state;
	int m_file_descriptor;

};

#endif // I2C_DRIVER_H
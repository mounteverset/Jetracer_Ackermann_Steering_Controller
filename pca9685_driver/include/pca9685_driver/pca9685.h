// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)

#ifndef PCA9685_H
#define PCA9685_H

#include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <bitset>


#include "pca9685_driver/pca9685_constants.h"

#include "i2c_driver/i2c_driver.h"

class PCA9685
{

private:
	uint8_t m_i2c_address;
	I2C_Driver * m_i2c_driver;

	// Keep track of the oscillator frequency
	unsigned int m_oscillator_frequency = PCA9685_OSCILLATOR_FREQUENCY_IN_HERTZ;

	// Keep track of the PWM frequency
	float m_pwm_frequency = PCA9685_PWM_DEFAULT_FREQUENCY_IN_HERTZ;

	// Keep track of the auto-increment mode
	bool m_auto_increment_enabled = false;

    // PCA9685 Functions

    bool read_register(uint8_t register_address, uint8_t * value);

	bool write_register(uint8_t register_address, uint8_t value, uint8_t num_attempts);

	bool write_pwm_pulse(uint8_t register_address, uint16_t on_count, uint16_t off_count, uint8_t num_attempts);

	bool write_pwm_full_on_or_full_off(uint8_t register_address, bool flag_on_off, uint8_t num_attempts);

	bool read_pwm_pulse_bytes(uint8_t register_address, uint16_t * on_bytes, uint16_t * off_bytes);

public:

    // Constructor
	PCA9685(I2C_Driver * i2c_driver);
	PCA9685(I2C_Driver * i2c_driver, uint8_t address);

	// Get & Set
	uint8_t get_i2c_address();
	bool set_i2c_address(uint8_t new_address);

    // RESET, SLEEP, WAKEUP
	bool reset();
	bool sleep();
	bool wakeup();

	// GET THE MODE 1 AND 2 BYTE
	bool get_mode1(uint8_t * mode1);
	bool get_mode2(uint8_t * mode2);

	// SET THE RESPONDS TO I2C SUB-ADDRESSES AND ALL CALL
	bool set_respond_to_i2c_bit(bool should_respond_sub_address_1, bool should_respond_sub_address_2, bool should_respond_sub_address_3, bool should_respond_all_call);

	// SET THE AUTO-INCREMENT BIT
	bool set_auto_increment_bit(bool should_auto_increment);

	// SET THE DEFAULTS FOR THE MODE 1 REGISTER
	bool set_mode1_defaults();

	// SET THE OUTPUT DRIVER MODE
	bool set_output_driver_mode(bool should_use_totem_pole_structure);

	// SET THE OUTPUT LOGIC INVERT MODE
	bool set_output_logic_invert_mode(bool should_use_inverted);

	// SET THE OUTPUTS CHANGE ON MODE
	bool set_output_change_on_mode(bool should_change_on_ack);

	// SET THE DEFAULTS FOR THE MODE 2 REGISTER
	// WHEN DRIVING SERVOS
	bool set_mode2_defaults_for_driving_servos();

	// SET AND GET THE PWM FREQUENCY
	bool set_pwm_frequency_in_hz(float freq_in_hz);
	bool get_pwm_frequency_in_hz_and_prescale(float * freq_in_hz, uint8_t * pre_scale);

	// SET THE PULSE OF A CHANNEL
	bool set_pwm_pulse(uint8_t channel_number, uint16_t on_count, uint16_t off_count);

	// SET THE PULSE OF A CHANNEL IN MICRO SECONDS
	bool set_pwm_pulse_in_microseconds(uint8_t channel_number, uint16_t pulse_with_in_microseconds);

	// SET A CHANNEL TO BE FULL ON OR FULL OFF
	bool set_pwm_full_on_or_full_off(uint8_t channel_number, bool flag_on_off);

	// GET THE PULSE DETAILS FOR A CHANNEL
	bool get_pwm_pulse(uint8_t channel_number, uint16_t * on_bytes, uint16_t * off_bytes);

	// TURN OFF ALL CHANNELS
	bool set_all_channels_full_off();

	// Convenience Function
    bool initialise_with_frequency_in_hz(float new_freq_in_hz, bool verbose);

}; // END OF CLASS DEFINITION

#endif // PCA9685_H

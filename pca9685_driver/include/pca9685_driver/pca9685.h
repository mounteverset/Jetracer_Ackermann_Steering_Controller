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
	/**
	 * @brief I2C Adress in hex
	 * 
	 */
	uint8_t m_i2c_address;

	/**
	 * @brief I2C_Drive used to communicate directly with the bus
	 * 
	 */
	I2C_Driver * m_i2c_driver;
	
	/**
	 * @brief Keep track of the oscillator frequency
	 * 
	 */
	unsigned int m_oscillator_frequency = PCA9685_OSCILLATOR_FREQUENCY_IN_HERTZ;
 
	/**
	 * @brief Keep track of the PWM frequency
	 * 
	 */
	float m_pwm_frequency = PCA9685_PWM_DEFAULT_FREQUENCY_IN_HERTZ;

	 
	/**
	 * @brief Keep track of the auto-increment mode
	 * 
	 */
	bool m_auto_increment_enabled = false;

	/**
	 * @brief PCA9685 Functions
	 * 
	 * @param register_address 
	 * @param value 
	 * @return true 
	 * @return false 
	 */
    bool read_register(uint8_t register_address, uint8_t * value);

	/**
	 * @brief PCA9685 Functions
	 * 
	 * @param register_address 
	 * @param value 
	 * @param num_attempts 
	 * @return true 
	 * @return false 
	 */
	bool write_register(uint8_t register_address, uint8_t value, uint8_t num_attempts);

	/**
	 * @brief PCA9685 Functions
	 * 
	 * @param register_address 
	 * @param on_count 
	 * @param off_count 
	 * @param num_attempts 
	 * @return true 
	 * @return false 
	 */
	bool write_pwm_pulse(uint8_t register_address, uint16_t on_count, uint16_t off_count, uint8_t num_attempts);

	/**
	 * @brief Completely set the PWM to be either completely on or off
	 * 
	 * @param register_address 
	 * @param flag_on_off 
	 * @param num_attempts 
	 * @return true 
	 * @return false 
	 */
	bool write_pwm_full_on_or_full_off(uint8_t register_address, bool flag_on_off, uint8_t num_attempts);

	/**
	 * @brief Read the number of bytes that are on for a PWM cycle
	 * 
	 * @param register_address 
	 * @param on_bytes 
	 * @param off_bytes 
	 * @return true 
	 * @return false 
	 */
	bool read_pwm_pulse_bytes(uint8_t register_address, uint16_t * on_bytes, uint16_t * off_bytes);

public:

    /**
     * @brief Construct a new PCA9685 object
     * 
     * @param i2c_driver 
     */
	PCA9685(I2C_Driver * i2c_driver);

	/**
	 * @brief Construct a new PCA9685 object
	 * 
	 * @param i2c_driver 
	 * @param address 
	 */
	PCA9685(I2C_Driver * i2c_driver, uint8_t address);

	/**
	 * @brief Get the i2c address object
	 * 
	 * @return uint8_t 
	 */
	uint8_t get_i2c_address();

	/**
	 * @brief Set the i2c address object
	 * 
	 * @param new_address 
	 * @return true 
	 * @return false 
	 */
	bool set_i2c_address(uint8_t new_address);

    /**
     * @brief Reset the PCA9685_Driver
     * 
     * @return true 
     * @return false 
     */
	bool reset();

	/**
	 * @brief Turn the PCA9685 into leep mode
	 * 
	 * @return true 
	 * @return false 
	 */
	bool sleep();

	/**
	 * @brief Wake the PCA9685 from sleep mode
	 * 
	 * @return true 
	 * @return false 
	 */
	bool wakeup();

	/**
	 * @brief Get the mode1 object
	 * 
	 * @param mode1 
	 * @return true 
	 * @return false 
	 */
	bool get_mode1(uint8_t * mode1);

	/**
	 * @brief Get the mode2 object
	 * 
	 * @param mode2 
	 * @return true 
	 * @return false 
	 */
	bool get_mode2(uint8_t * mode2);

	 
	/**
	 * @brief Set the response to I2C sub-adresses and all call
	 * 
	 * @param should_respond_sub_address_1 
	 * @param should_respond_sub_address_2 
	 * @param should_respond_sub_address_3 
	 * @param should_respond_all_call 
	 * @return true 
	 * @return false 
	 */
	bool set_respond_to_i2c_bit(bool should_respond_sub_address_1, bool should_respond_sub_address_2, bool should_respond_sub_address_3, bool should_respond_all_call);

	/**
	 * @brief Set the auto increment bit
	 * 
	 * @param should_auto_increment 
	 * @return true 
	 * @return false 
	 */
	bool set_auto_increment_bit(bool should_auto_increment);

	/**
	 * @brief Set the defaults for the mode1 register
	 * 
	 * @return true 
	 * @return false 
	 */
	bool set_mode1_defaults();

	/**
	 * @brief Set the output driver mode
	 * 
	 * @param should_use_totem_pole_structure 
	 * @return true 
	 * @return false 
	 */
	bool set_output_driver_mode(bool should_use_totem_pole_structure);

	/**
	 * @brief Set the output logic invert mode
	 * 
	 * @param should_use_inverted 
	 * @return true 
	 * @return false 
	 */
	bool set_output_logic_invert_mode(bool should_use_inverted);

	/**
	 * @brief Set the output change on mode
	 * 
	 * @param should_change_on_ack 
	 * @return true 
	 * @return false 
	 */
	bool set_output_change_on_mode(bool should_change_on_ack);

	// SET THE DEFAULTS FOR THE MODE 2 REGISTER
	// WHEN DRIVING SERVOS
	/**
	 * @brief Set the defaults mode2 register when driving the servos
	 * 
	 * @return true 
	 * @return false 
	 */
	bool set_mode2_defaults_for_driving_servos();

	/**
	 * @brief Set the pwm frequency in hz
	 * 
	 * @param freq_in_hz 
	 * @return true 
	 * @return false 
	 */
	bool set_pwm_frequency_in_hz(float freq_in_hz);

	/**
	 * @brief Get the pwm frequency in hz and prescale
	 * 
	 * @param freq_in_hz 
	 * @param pre_scale 
	 * @return true 
	 * @return false 
	 */
	bool get_pwm_frequency_in_hz_and_prescale(float * freq_in_hz, uint8_t * pre_scale);

	/**
	 * @brief Set the pwm pulse
	 * 
	 * @param channel_number 
	 * @param on_count 
	 * @param off_count 
	 * @return true 
	 * @return false 
	 */
	bool set_pwm_pulse(uint8_t channel_number, uint16_t on_count, uint16_t off_count);

	/**
	 * @brief Set the pwm pulse in microseconds
	 * 
	 * @param channel_number 
	 * @param pulse_with_in_microseconds 
	 * @return true 
	 * @return false 
	 */
	bool set_pwm_pulse_in_microseconds(uint8_t channel_number, uint16_t pulse_with_in_microseconds);

	/**
	 * @brief Set the pwm full on or full off
	 * 
	 * @param channel_number 
	 * @param flag_on_off 
	 * @return true 
	 * @return false 
	 */
	bool set_pwm_full_on_or_full_off(uint8_t channel_number, bool flag_on_off);

	/**
	 * @brief Get the pwm pulse object
	 * 
	 * @param channel_number 
	 * @param on_bytes 
	 * @param off_bytes 
	 * @return true 
	 * @return false 
	 */
	bool get_pwm_pulse(uint8_t channel_number, uint16_t * on_bytes, uint16_t * off_bytes);

	/**
	 * @brief Set the all channels full off
	 * 
	 * @return true 
	 * @return false 
	 */
	bool set_all_channels_full_off();

	/**
	 * @brief Convenience Function
	 * 
	 * @param new_freq_in_hz 
	 * @param verbose 
	 * @return true 
	 * @return false 
	 */
    bool initialise_with_frequency_in_hz(float new_freq_in_hz, bool verbose);

}; // END OF CLASS DEFINITION

#endif // PCA9685_H

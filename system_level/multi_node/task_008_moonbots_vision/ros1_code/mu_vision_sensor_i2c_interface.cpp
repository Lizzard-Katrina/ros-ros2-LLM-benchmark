/*
 * mu_vision_sensor_uart_interface.cpp
 *
 *  Created on: 2018年8月8日
 *      Author: ysq
 */

#include "mu_vision_sensor_i2c_interface.h"

MuVsI2CMethod::MuVsI2CMethod(uint32_t address)
    : MuVsMethod() {
  mu_address_ = address;
}

MuVsI2CMethod::~MuVsI2CMethod() {}

//Not use
//mu_err_t MuVsI2CMethod::begin(uint32_t speed) {
//  mu_err_t err;
//
//  return err;
//}

mu_err_t MuVsI2CMethod::Get(const uint8_t reg_address,
                             uint8_t* value) {
	return I2CRead(reg_address, value);
}

mu_err_t MuVsI2CMethod::Set(const uint8_t reg_address,
                            const uint8_t value) {
  return I2CWrite(reg_address, value);
}

mu_err_t MuVsI2CMethod::Read(MuVsMessageVisionType vision_type,
                             MuVsVisionState* vision_state) {
//TODO：
// Write the vision_type to the sensor register (I2CWrite kRegVisionId)
//Read the number of detected objects from kRegResultNumber
// Read the frame count from kRegFrameCount
//- Loop over each detected object:
//    - Populate vision_state->vision_result[i]
//- Return appropriate mu_err_t error code on failure
//END TODO
}





#include "LSM303C.h"

/**
 * Start up communications with the sensor.
 * This should be called before applying the sensor's configuration.
 *
 * @return: True if communications started successfully.
 */
bool LSM303C::begin(uint8_t comms_mode, uint8_t xl_cs_pin, uint8_t mag_cs_pin) {
    _comms_mode = comms_mode;
    if (_comms_mode == LSM303C_SPI_MODE) {
        _xl_chip_select_pin = xl_cs_pin;
        _mag_chip_select_pin = mag_cs_pin;
    }

    return xl_comms_check();
}

/**
 * Check the status of communications with the sensor.
 * The ID registers from the accelerometer and magnetometer registers are read.
 * Both IDs must match for the check to pass.
 *
 * @return: True if check passes; False if sensor IDs do not match known values or cannot be read.
 */
bool LSM303C::xl_comms_check() {
    uint8_t xl_id;
    read(&xl_id, LSM303C_I2C_ADDRESS_XL, WHO_AM_I);

    return xl_id == LSM303C_XL_ID;
}

/**
 * Check the status of communications with the accelerometer.
 * The ID registers from the accelerometer register is read.
 *
 * @return: True if check passes; False if sensor IDs do not match known values or cannot be read.
 */
bool LSM303C::mag_comms_check() {
    uint8_t mag_id;
    read(&mag_id, LSM303C_I2C_ADDRESS_MAG, WHO_AM_I);

    return mag_id == LSM303C_MAG_ID;
}

/**
 * Write to a register
 * @param input: Start of the data to be transmitted to the device.
 * @param device_address: Device to be written to [XL/MAG]. Also selects cs pin if using SPI mode.
 * @param address: Register address to be written to.
 * @param length: Number of bytes to write.
 * @return: True if successful.
 */
bool LSM303C::write(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == LSM303C_I2C_MODE)
        success = write_i2c(input, device_address, address, length);
    else if (_comms_mode == LSM303C_SPI_MODE)
        success = write_spi(input, device_address, address, length);
    return success;
}

/**
 * Read from a register
 * @param output: Start of the container to store data read from the device.
 * @param device_address: Device to be read from [XL/MAG]. Also selects cs pin if using SPI mode.
 * @param address: Register address to be read from.
 * @param length: Number of bytes to read.
 * @return: True if successful.
 */
bool LSM303C::read(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == LSM303C_I2C_MODE) {
        success = read_i2c(output, device_address, address, length);
    } else if (_comms_mode == LSM303C_SPI_MODE) {
        success = read_spi(output, device_address, address, length);
    }
    return success;
}

/**
 * Write to a register using I2C
 * @param input: Start of the data to be transmitted to the device.
 * @param device_address: Device to be written to [XL/MAG].
 * @param address: Register address to be written to.
 * @param length: Number of bytes to write.
 * @return: True if successful.
 */
bool LSM303C::write_i2c(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(device_address);
    Wire.write(address);
    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        result = false;
    }
    return result;
}

/**
 * Read from a register using I2C
 * @param output: Start of the container to store data read from the device.
 * @param device_address: Device to be read from [XL/MAG].
 * @param address: Register address to be read from.
 * @param length: Number of bytes to read.
 * @return: True if successful.
 */
bool LSM303C::read_i2c(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(device_address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(uint8_t(device_address), length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

/**
 * Write to a register using SPI
 * @param input: Start of the data to be transmitted to the device.
 * @param device_address: Device to be written to [XL/MAG].
 * @param address: Register address to be written to.
 * @param length: Number of bytes to write.
 * @return: True if successful.
 */
bool LSM303C::write_spi(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool success = false;
    int8_t cs_pin = -1;
    if (device_address == LSM303C_I2C_ADDRESS_XL)
        cs_pin = _xl_chip_select_pin;
    else if (device_address == LSM303C_I2C_ADDRESS_MAG)
        cs_pin = _mag_chip_select_pin;

    if (cs_pin > 0) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(address);
        for (size_t i = 0; i < length; i++) {
            SPI.transfer(input[i]);
        }
        digitalWrite(cs_pin, HIGH);
        success = true;
    }

    return success;
}

/**
 * Read from a register using SPI
 * @param output: Start of the container to store data read from the device.
 * @param device_address: Device to be read from [XL/MAG].
 * @param address: Register address to be read from.
 * @param length: Number of bytes to read.
 * @return: True if successful.
 */
bool LSM303C::read_spi(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length) {
    bool success = false;
    uint8_t num_empty_bytes = 0;
    int8_t cs_pin = -1;

    if (device_address == LSM303C_I2C_ADDRESS_XL)
        cs_pin = _xl_chip_select_pin;
    else if (device_address == LSM303C_I2C_ADDRESS_MAG)
        cs_pin = _mag_chip_select_pin;

    // Proceed if a valid pin has been selected
    if (cs_pin > 0) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(address | 0x80 | 0x40);
        for (size_t i = 0; i < length; i++) {
            uint8_t c = SPI.transfer(0x00);
            if (c == 0xFF) num_empty_bytes++;
            *output = c;
            output++;
        }
        if (num_empty_bytes != length) success = true;
        digitalWrite(cs_pin, HIGH);
    }

    return success;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_inactive_threshold_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, ACT_THS); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_inactive_duration_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, ACT_DUR); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_rate_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG1); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_filter_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG2); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_interrupt_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG3); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_scale_bw_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG4); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_int_dec_debug_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG5); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_reboot_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG6); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_xl_int_mode_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG7); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_fifo_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, FIFO_CTRL); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_temp_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG1); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_scale_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG2); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_mode_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG3); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_data_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG4); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_bdu_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG5); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 */
void LSM303C::write(lsm303c_mag_interrupt_config_t config) { write((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, INT_CFG); }

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 * @param int_number: Interrupt generator number to write configuration to.
 */
void LSM303C::write(lsm303c_xl_intgen_config_t config, uint8_t int_number) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 1)
        reg_address = IG_CFG1;
    else if (int_number == 2)
        reg_address = IG_CFG2;
    if (reg_address != 0) write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 * @param int_number: Interrupt generator number to write configuration to.
 */
void LSM303C::write(lsm303c_xl_intgen_duration_config_t config, uint8_t int_number) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 1)
        reg_address = IG_DUR1;
    else if (int_number == 2)
        reg_address = IG_DUR2;
    if (reg_address != 0) write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

/**
 * Write configuration to the device.
 * @param config: Configuration type and data to be written.
 * @param int_number: Interrupt generator number to write configuration to.
 * @param axis: Axis to write threshold for.
 */
void LSM303C::write(lsm303c_xl_intgen_threshold_config_t config, uint8_t int_number, char axis) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 2)
        reg_address = IG_THS2;
    else if (int_number == 1) {
        if (axis == 'x')
            reg_address = IG_THS_X1;
        else if (axis == 'y')
            reg_address = IG_THS_Y1;
        else if (axis == 'z')
            reg_address = IG_THS_Z1;
    }
    if (reg_address != 0) write((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_inactive_threshold_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, ACT_THS); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_inactive_duration_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, ACT_DUR); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_rate_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG1); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_filter_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG2); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_interrupt_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG3); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_scale_bw_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG4); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_int_dec_debug_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG5); }

/**
 * Configurations are type-specific.
 * Read configuration information from the device.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_reboot_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG6); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_xl_int_mode_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, CTRL_REG7); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_fifo_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, FIFO_CTRL); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_temp_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG1); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_scale_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG2); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_mode_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG3); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_data_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG4); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_bdu_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, CTRL_REG5); }

/**
 * Configurations are type-specific.
 * Read configuration information from the device.
 * @param config: Container for the configuration to be read into.
 */
void LSM303C::read(lsm303c_mag_interrupt_config_t& config) { read((uint8_t*)&config, LSM303C_I2C_ADDRESS_MAG, INT_CFG); }

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 * @param int_number: Interrupt generator number to read from.
 */
void LSM303C::read(lsm303c_xl_intgen_config_t& config, uint8_t int_number) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 1)
        reg_address = IG_CFG1;
    else if (int_number == 2)
        reg_address = IG_CFG2;
    if (reg_address != 0) read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 * @param int_number: Interrupt generator number to read from.
 */
void LSM303C::read(lsm303c_xl_intgen_duration_config_t& config, uint8_t int_number) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 1)
        reg_address = IG_DUR1;
    else if (int_number == 2)
        reg_address = IG_DUR2;
    if (reg_address != 0) read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

/**
 * Read configuration information from the device.
 * Configurations are type-specific.
 * @param config: Container for the configuration to be read into.
 * @param int_number: Interrupt generator number to read from.
 * @param axis: Axis to read threshold from.
 */
void LSM303C::read(lsm303c_xl_intgen_threshold_config_t& config, uint8_t int_number, char axis) {
    lsm303c_reg_t reg_address = lsm303c_reg_t(0);
    if (int_number == 2)
        reg_address = IG_THS2;
    else if (int_number == 1) {
        if (axis == 'x')
            reg_address = IG_THS_X1;
        else if (axis == 'y')
            reg_address = IG_THS_Y1;
        else if (axis == 'z')
            reg_address = IG_THS_Z1;
    }
    if (reg_address != 0) read((uint8_t*)&config, LSM303C_I2C_ADDRESS_XL, reg_address);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read raw acceleration data from the device.
 * @param data: Container to put a single raw read into.
 */
void LSM303C::read(lsm303c_xl_raw_data_t& data) {
    uint8_t buffer[6];
    read(buffer, LSM303C_I2C_ADDRESS_XL, OUT_X_L, 6);
    data.x = buffer[0] | uint16_t(buffer[1]) << 8;
    data.y = buffer[2] | uint16_t(buffer[3]) << 8;
    data.z = buffer[4] | uint16_t(buffer[5]) << 8;
}

/**
 * Read corrected acceleration data from the device.
 * The output of each axis reading is in mg.
 * @param data: Container for the corrected acceleration information.
 */
void LSM303C::read(lsm303c_xl_corrected_data_t& data) {
    lsm303c_xl_raw_data_t raw;
    lsm303c_xl_scale_bw_config_t config;
    read(raw);
    read(config);

    uint8_t scale_code = config.xl_scale;
    float scale = 0.0;
    if (scale_code == LSM303C_XL_SCALE_2G)
        scale = 2.0;
    else if (scale_code == LSM303C_XL_SCALE_4G)
        scale = 4.0;
    else if (scale_code == LSM303C_XL_SCALE_8G)
        scale = 8.0;

    data.x = (float(raw.x) * scale) / 32768.0;
    data.y = (float(raw.y) * scale) / 32768.0;
    data.z = (float(raw.z) * scale) / 32768.0;
}

/**
 * Read acceleration data from the device in the form of roll and pitch.
 * The readings are taken with respect to gravity, assuming that the device is stationary.
 * Readings are in the form of degrees.
 * @param data: Container to put the read data into.
 */
void LSM303C::read(lsm303c_xl_roll_pitch_data_t& data) {
    lsm303c_xl_corrected_data_t corrected_data;
    read(corrected_data);

    // Data needs to be in Gs for roll/pitch/yaw calculations
    float x = corrected_data.x / 1000.0;
    float y = corrected_data.y / 1000.0;
    float z = corrected_data.z / 1000.0;

    // Calculate z-sign for roll correction
    int8_t z_sign;
    if (z >= 0)
        z_sign = 1;
    else
        z_sign = -1;

    float total = sqrt(x * x + y * y + z * z);  // TODO - gate roll/pitch if total acceleration is too high/low

    // See Freescale Application Note AN3461 - Tilt Sensing Using a Three-Axis Accelerometer
    data.pitch = atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;                  // Eq. 26
    data.roll = atan2(y, z_sign * sqrt(z * z + (x * x * 1E-6))) * RAD_TO_DEG;  // Eq. 38
}

/**
 * Read raw magnetometer and temperature data from the device.
 * @param data: Container to put a single raw reading into.
 */
void LSM303C::read(lsm303c_mag_raw_data_t& data) {
    uint8_t buffer[8];
    read(buffer, LSM303C_I2C_ADDRESS_MAG, OUT_X_L, LSM303C_MAG_READ_SIZE);
    data.x = buffer[0] | uint16_t(buffer[1]) << 8;
    data.y = buffer[2] | uint16_t(buffer[3]) << 8;
    data.z = buffer[4] | uint16_t(buffer[5]) << 8;
}

/**
 * Read corrected magnetometer and temperature data from the device.
 * Magnetometer readings are in milli-gauss.
 * Temperature readings are in degrees C.
 * @param data: Container for the corrected magnetometer and temperature data.
 */
void LSM303C::read(lsm303c_mag_corrected_data_t& data) {
    lsm303c_mag_raw_data_t raw;
    read(raw);

    data.x = float(raw.x) * LSM303C_MAG_SCALAR;
    data.y = float(raw.y) * LSM303C_MAG_SCALAR;
    data.z = float(raw.z) * LSM303C_MAG_SCALAR;
}

void LSM303C::read(lsm303c_temp_data_t& data) {
    uint8_t buffer[2];
    read(buffer, LSM303C_I2C_ADDRESS_MAG, LSM303C_REGISTER::TEMP_L, LSM303C_TEMPERATURE_READ_SIZE);
    data.raw = (int16_t(buffer[1]) << 8) | buffer[0];
    data.temperature = float(data.raw) / 8.0 + 25.0;
}

float LSM303C::get_heading(bool tilt_correct_enabled) {
    float heading;
    lsm303c_mag_corrected_data_t mag_data;
    read(mag_data);

    if (tilt_correct_enabled) {
        lsm303c_xl_roll_pitch_data_t roll_pitch_data;
        read(roll_pitch_data);

        float roll = roll_pitch_data.roll * DEG_TO_RAD;
        float pitch = roll_pitch_data.pitch * DEG_TO_RAD;

        float x_corrected = mag_data.x * cos(roll) + mag_data.z * sin(roll);
        float y_corrected = mag_data.y * cos(pitch) + mag_data.y * sin(roll) * sin(pitch) - mag_data.z * cos(roll) * sin(pitch);

        heading = atan2(y_corrected, x_corrected) * RAD_TO_DEG;

    } else {
        // Assume the sensor is just lying flat

        // Avoid division by zero if X is 0
        if (mag_data.x != 0) heading = atan2(mag_data.y, mag_data.x) * RAD_TO_DEG;

        // X = 0
        else {
            if (mag_data.y < 0)
                heading = 90.0;
            else
                heading = 0.0;
        }
    }

    // Correct for negative headings. Not sure if necessary
    if (heading < 0) {
        heading += 360.0;
    }

    return heading;
}

void LSM303C::read(lsm303c_orientation_data_t& data) {
    lsm303c_xl_roll_pitch_data_t roll_pitch;
    read(roll_pitch);
    data.pitch = roll_pitch.pitch;
    data.roll = roll_pitch.roll;
    data.yaw = get_heading(true);
}

float LSM303C::get_temperature() {
    lsm303c_temp_data_t data;
    read(data);
    return data.temperature;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read in the contents of the FIFO buffer.
 * The FIFO buffer can store up to 32 levels of recorded accelerometer data.
 * Each reading is 6 bytes; 2 bytes for each axis.
 * @param data Array of containers to read raw readings into. Should be 32 readings deep.
 * @param max_measurements The maximum number of levels to read from the FIFO cache.
 * @return: Number of readings taken from the FIFO buffer.
 */
uint8_t LSM303C::read(lsm303c_xl_raw_data_t data[], uint8_t max_measurements) {
    uint8_t buffer[LSM303C_XL_READ_SIZE * LSM303C_FIFO_CAPACITY];
    lsm303c_fifo_status_t fifo;

    // Calculate the number of bytes to be read from the buffer
    read(fifo);
    if (fifo.level > max_measurements) fifo.level = max_measurements;
    uint16_t read_size = LSM303C_XL_READ_SIZE * fifo.level;
    if (read_size > 0) {
        read(buffer, LSM303C_I2C_ADDRESS_XL, OUT_X_L, read_size);
    }

    for (size_t i = 0; i < fifo.level; i++) {
        data[i].x = buffer[i * LSM303C_XL_READ_SIZE] | (uint16_t(buffer[i * LSM303C_XL_READ_SIZE + 1]) << 8);
        data[i].y = buffer[i * LSM303C_XL_READ_SIZE + 2] | (uint16_t(buffer[i * LSM303C_XL_READ_SIZE + 3]) << 8);
        data[i].z = buffer[i * LSM303C_XL_READ_SIZE + 4] | (uint16_t(buffer[i * LSM303C_XL_READ_SIZE + 5]) << 8);
    }
    return fifo.level;
}

///////////////////////////////////////////////////////////////////////////////

void LSM303C::read(lsm303c_xl_status_t& status) { read((uint8_t*)&status, LSM303C_I2C_ADDRESS_XL, STATUS_REG); }

void LSM303C::read(lsm303c_fifo_status_t& status) { read((uint8_t*)&status, LSM303C_I2C_ADDRESS_XL, FIFO_SRC); }

void LSM303C::read(lsm303c_mag_status_t& status) { read((uint8_t*)&status, LSM303C_I2C_ADDRESS_MAG, STATUS_REG); }

void LSM303C::read(lsm303c_mag_interrupt_status_t& status) { read((uint8_t*)&status, LSM303C_I2C_ADDRESS_MAG, INT_SRC); }

void LSM303C::read(lsm303c_xl_intgen_status_t& status, uint8_t int_number) {
    uint8_t reg_address = IG_SRC1;
    if (int_number == 2) reg_address = IG_SRC2;
    read((uint8_t*)&status, LSM303C_I2C_ADDRESS_XL, lsm303c_reg_t(reg_address));
}

uint8_t LSM303C::get_odr_code(float odr) {
    uint8_t odr_code = 0;
    for (size_t i = 0; i < sizeof(LSM303C_XL_ODR_HZ) / sizeof(LSM303C_XL_ODR_HZ[0]); i++) {
        if (odr == LSM303C_XL_ODR_HZ[i]) {
            odr_code = i;
            break;
        }
    }

    return odr_code;
}

uint8_t LSM303C::get_full_scale_range_code(uint8_t range) {
    uint8_t range_code = 0;
    for (size_t i = 0; i < sizeof(LSM303C_XL_FULL_RANGE_SCALES) / sizeof(LSM303C_XL_FULL_RANGE_SCALES[0]); i++) {
        if (range == LSM303C_XL_FULL_RANGE_SCALES[i]) {
            range_code = i;
            break;
        }
    }

    return range_code;
}

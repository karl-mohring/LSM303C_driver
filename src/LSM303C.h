#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifndef LSM303C_H
#define LSM303C_H

const uint8_t LSM303C_XL_ID = 0x41;
const uint8_t LSM303C_MAG_ID = 0x3D;
const float LSM303C_MAG_SCALAR = 0.58;  // milli-gauss per adc level.
const float LSM303C_TEMPERATURE_SCALAR = 1.0 / 8.0;
const uint8_t LSM303C_FIFO_CAPACITY = 32;

/**
 * Communication modes for the device [I2C, SPI]
 */
enum LSM303C_COMMS_MODE { LSM303C_I2C_MODE, LSM303C_SPI_MODE };

/**
 * I2C Addresses of each device section.
 * They are separate for whatever reason and fixed.
 */
typedef enum LSM303C_I2C_ADDRESS { LSM303C_I2C_ADDRESS_XL = 0x1D, LSM303C_I2C_ADDRESS_MAG = 0x1E } lsm303c_device_address_t;

enum LSM303C_DATA_READ_SIZE { LSM303C_XL_READ_SIZE = 6, LSM303C_READ_SIZE_SINGLE = 2, LSM303C_MAG_READ_SIZE = 6, LSM303C_TEMPERATURE_READ_SIZE = 2 };

///////////////////////////////////////////////////////////////////////////////
// * Accelerometer definitions
///////////////////////////////////////////////////////////////////////////////
// ACT_THS_A

/**
 * Activity/Inactivity threshold for auto wake/sleep.
 * The threshold value is the full-scale range / 128 mg.
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t threshold : 7;
        bool _reserved : 1;
    };
} lsm303c_xl_inactive_threshold_t;

///////////////////////////////////////////////////////////////////////////////
// ACT_DUR_A

/**
 * Time that the sensor must remain inactive before entering sleep.
 * Time is given as 8*duration/ODR seconds.
 * e.g.: A setting of 50, with the sensor operating at 100 Hz, would result in a inactivity time of 2.8 seconds.
 */
typedef union {
    uint8_t duration;
} lsm303c_xl_inactive_duration_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG1_A

/**
 * Modes for the block data update setting.
 * If BDU is enabled, data in the output registers is held once a register read has started.
 * Data is prevented from being overwritten to avoid mid-read changes, which may lead to incorrect readings.
 */
enum LSM303C_BDU_STATE { LSM303C_BDU_CONTINUOUS_UPDATE = 0, LSM303C_BDU_UPDATE_AFTER_READ = 1 };

/**
 * Output data rate for the accelerometer.
 * This rate reflects the rate at which the sensor is sampled.
 * Sample decimation is possible to take samples at rates lower than any of the given rates.
 */
enum LSM303C_XL_ODR {
    LSM303C_XL_ODR_POWER_DOWN = 0,
    LSM303C_XL_ODR_10HZ = 1,
    LSM303C_XL_ODR_50HZ = 2,
    LSM303C_XL_ODR_100HZ = 3,
    LSM303C_XL_ODR_200HZ = 4,
    LSM303C_XL_ODR_400HZ = 5,
    LSM303C_XL_ODR_800HZ = 6
};

const uint16_t LSM303C_XL_ODR_HZ[] = {0, 10, 50, 100, 200, 400, 800};

/**
 * Config for CTRL_REG1_A.
 *
 * Includes:
 *  - Per-axis enable
 *  - Blocking data updates mid-read
 *  - Sample rate
 *  - High resolution mode
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t x_axis_enabled : 1;             // 1: Enable axis
        uint8_t y_axis_enabled : 1;             // 1: Enable axis
        uint8_t z_axis_enabled : 1;             // 1: Enable axis
        uint8_t block_data_update_enabled : 1;  // 0: Continuous update; 0: Update registers after read
        uint8_t output_data_rate : 3;           // 0: Power down; 1-6: 10-800 Hz
        uint8_t high_resolution_enabled : 1;    // 1: HR mode enabled
    };
} lsm303c_xl_rate_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG2_A

/**
 * Reference selection for the high-pass filter.
 * Normal mode uses he internal reference.
 * Referenced mode uses an external signal for filtering.
 */
enum LSM303C_HPF_MODE { LSM303C_HPF_MODE_NORMAL = 0x00, LSM303C_HPF_MODE_REFERENCED = 0x01 };

/**
 * Low pass cutoff frequency for high-resolution mode.
 * "High resolution" seems to mean "LPF enabled".
 * Not sure what else flicking the accelerometer in high resolution mode does.
 * The datasheet is a bit stingy with details.
 *
 * Anyway, looks like the low-pass filter is intended for oversampling.
 * It's a shame the cutoff frequencies don't also correspond with the decimation rates.
 */
enum LSM303C_LPF_CUTOFF { LSM303C_LPF_CUTOFF_ODR_DIV_50 = 0, LSM303C_LPF_CUTOFF_ODR_DIV_100 = 1, LSM303C_LPF_CUTOFF_ODR_DIV_9 = 2, LSM303C_LPF_CUTOFF_ODR_DIV_400 = 3 };

/**
 * Configuration for CTRL_REG2_A.
 * Includes:
 *  - High-pass filter settings for interrupts
 *  - Low-pass cutoff for high-resolution mode
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t interrupt1_high_pass_filter_enabled : 1;          // High-pass filter enable for interrupt1
        uint8_t interrupt2_high_pass_filter_enabled : 1;          // High pass filter enable for interrupt2
        uint8_t internal_high_pass_filter_enabled : 1;            // 0: internal filter bypassed; 1: output data filtered
        uint8_t high_pass_filter_external_reference_enabled : 2;  // High pass filter mode (LSM303C_HPF_MODE)
        uint8_t low_pass_filter_cutoff : 2;                       // High pass filter cutoff frequency selection (LSM303C_LPF_CUTOFF)
    };
} lsm303c_xl_filter_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG3_A

/**
 * Configuration for CTRL_REG3_A.
 *
 * Configures:
 *  - Interrupts for data ready
 *  - FIFO interrupts
 *  - Interrupts from generators
 *  - Inactivity interrupts
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t data_ready_interrupt_enabled : 1;      // Data read signal on xl interrupt
        uint8_t fifo_threshold_interrupt_enabled : 1;  // FIFO threshold signal on xl interrupt
        uint8_t fifo_overrun_interrupt_enabled : 1;    // FIFO overrun signal on xl interrupt
        uint8_t interrupt1_enabled : 1;                // Interrupt generator enable on interrupt 1
        uint8_t interrupt2_enabled : 1;                // Interrupt generator enabled on interrupt 2
        uint8_t inactivity_interrupt_enabled : 1;      // Inactivity interrupt enabled for xl
        uint8_t fifo_stop_on_threshold_enabled : 1;    // Stop FIFO buffer filling once threshold reached
        uint8_t fifo_enabled : 1;                      // Enable FIFO buffer
    };
} lsm303c_xl_interrupt_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG4_A

/**
 * Full-range scale of the accelerometer.
 * Each setting represents the ± maximum reading.
 */
enum LSM303C_XL_SCALE { LSM303C_XL_SCALE_2G = 0, LSM303C_XL_SCALE_4G = 2, LSM303C_XL_SCALE_8G = 3 };

const uint8_t LSM303C_XL_FULL_RANGE_SCALES[] = {2, 4, 8};

/**
 * Bandwidth of the anti-aliasing filter for acceleration measurements.
 * The bandwidth is always 400 Hz for sampling rates of 10 Hz and 50 Hz.
 */
enum LSM303C_AA_BANDWIDTH { LSM303C_AA_BW_400HZ = 0, LSM303C_AA_BW_200HZ = 1, LSM303C_AA_BW_100HZ = 2, LSM303C_AA_BW_50HZ = 3 };

/**
 * Configuration for CTRL_REG4_A.
 * Includes:
 *  - Comms settings for I2C/SPI
 *  - AA filter bandwidth
 *  - Sensor scale
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t spi_reads_enabled : 1;               // 0: SPI write only; 1: SPI read and write enabled
        uint8_t i2c_disabled : 1;                    // 0: Enable I2C comms; 1: Disable I2C
        uint8_t auto_increment_address_enabled : 1;  // Auto-increment address during multi-byte access
        uint8_t auto_select_aa_bandwidth : 1;        // 0: AA bandwidth auto; 1: AA set manually using 'aa_bandwidth'
        uint8_t xl_scale : 2;                        // Full-scale selection for the xl (LSM303C_XL_SCALE)
        uint8_t aa_bandwidth : 2;                    // Bandwidth setting for the AA filter (LSM303C_AA_BW)
    };
} lsm303c_xl_scale_bw_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG5_A

/**
 * Output configuration for the accelerometer interrupt pin.
 * Pins in open drain mode need a pullup or pulldown resistor, depending on interrupt polarity.
 */
enum LSM303C_INTERRUPT_OUTPUT_MODE { LSM303C_INT_PUSH_PULL = 0, LSM303C_INT_OPEN_DRAIN = 1 };

/**
 * Self test modes.
 * TODO - self-test mode explanation
 */
enum LSM303C_SELF_TEST_MODE { LSM303C_SELF_TEST_NORMAL = 0, LSM303C_POSITIVE_SELF_TEST = 1, LSM303C_NEGATIVE_SELF_TEST = 2 };

/**
 * Accelerometer reading decimation mode.
 * Decimation refers to how often the output registers or FIFO buffer is update in comparison to sample rate.
 * A decimation of 8 means that the regisers are only updated every 8 reads.
 */
enum LSM303C_DECIMATION_MODE { LSM303C_DEC_NONE = 0, LSM303C_DEC_X2 = 1, LSM303C_DEC_X4 = 2, LSM303C_DEC_X8 = 3 };

const uint8_t LSM303C_DECIMATION_FACTORS[] = {1, 2, 4, 8};

/**
 * Configuration for CTRL_REG5_A
 * Includes:
 *  - Interrupt mode and polarity
 *  - Self-test mode and debug
 *  - Decimation
 *  - Soft reset of configuration registers
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t interrupt_output_mode : 1;  // 0: Push-pull; 1: Open drain (LSM303C_INTERRUPT_OUTPUT_MODE)
        uint8_t interrupt_active_low_enabled : 1;
        uint8_t self_test_mode : 2;
        uint8_t decimation_mode : 2;
        uint8_t trigger_soft_reset : 1;
        uint8_t debug_enabled : 1;
    };
} lsm303c_xl_int_dec_debug_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG6_A

/**
 * "Configuration" for CTRL_REG6_A
 * Contains the reboot trigger. Nothing else.
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved : 7;
        uint8_t force_reboot_enabled : 1;
    };
} lsm303c_xl_reboot_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG7_A

/**
 * FIFO modes.
 * Bypass - FIFO buffer is off and data goes straight to the registers. Overwritten data is lost.
 * FIFO - Data is written to the buffer and stops when the buffer is full.
 * Stream - Data is written to the buffer. Old data is overwritten when the buffer is full.
 */
enum LSM303C_FIFO_MODE {
    LSM303C_FIFO_BYPASS = 0,
    LSM303C_FIFO_ON = 1,
    LSM303C_FIFO_STREAM = 2,
    LSM303C_FIFO_STREAM_TO_FIFO = 3,    // Int1 active: FIFO; inactive: stream
    LSM303C_FIFO_BYPASS_TO_STREAM = 4,  // Int1 active: stream; inactive: bypass
    LSM303C_FIFO_BYPASS_TO_FIFO = 7     // Int1 active: FIFO; inactive: bypass
};

/**
 * Configuration for CTRL_REG7_A.
 * Includes:
 *  - 4D recognition for interrupts
 *  - Interrupt latch mode
 *  - Interrupt duration reset enable
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t interrupt1_4d_enabled : 1;                      // Enable 4D position recognition for interrupt generator 1
        uint8_t interrupt2_4d_enabled : 1;                      // Enable 4D position recognition for interrupt generator 2
        uint8_t interrupt1_latch_enabled : 1;                   // Latch interrupt request. Cleared by reading IG_SRC
        uint8_t interrupt2_latch_enabled : 1;                   // Latch interrupt request. Cleared by reading IG_SRC
        uint8_t interrupt1_duration_counter_reset_enabled : 1;  // Duration counter mode (LSM303C_DURATION_COUNTER_MODE)
        uint8_t interrupt2_duration_counter_reset_enabled : 1;  // Duration counter mode (LSM303C_DURATION_COUNTER_MODE)
    };
} lsm303c_xl_int_mode_config_t;

///////////////////////////////////////////////////////////////////////////////
// STATUS_REG_A

/**
 * Status from STATUS_REG_A.
 * Includes:
 *  - Data ready status per axis
 *  - Data overrun status per axis
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t data_available_x : 1;
        uint8_t data_available_y : 1;
        uint8_t data_available_z : 1;
        uint8_t data_available_all : 1;
        uint8_t data_overrun_x : 1;
        uint8_t data_overrun_y : 1;
        uint8_t data_overrun_z : 1;
        uint8_t data_overrun_any : 1;
    };
} lsm303c_xl_status_t;

///////////////////////////////////////////////////////////////////////////////
// OUT_x_x_A

/**
 * Accelerometer data per reading.
 * Each reading contains data from all 3 axes.
 * Readings are signed.
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303c_xl_raw_data_t;

/**
 * Accelerometer data per reading.
 * Each reading contains data from all 3 axes in mg.
 * Readings are signed.
 */
typedef struct {
    float x;
    float y;
    float z;
} lsm303c_xl_corrected_data_t;

/**
 * Accelerometer data in the format of roll and pitch with respect to gravity.
 */
typedef struct {
    float roll;
    float pitch;
} lsm303c_xl_roll_pitch_data_t;

/**
 * Device orientation in roll/pitch/yaw with respect to gravity.
 * Both accelerometer and magnetometer need to be active to successfully calculate the values.
 * All outputs are given in degrees.
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} lsm303c_orientation_data_t;

///////////////////////////////////////////////////////////////////////////////
// FIFO_CTRL

/**
 * FIFO configuration for FIFO_CTRL.
 * Includes:
 *  - FIFO operating mode
 *  - Threshold for FIFO interrupts
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t threshold : 5;  // FIFO level threshold
        uint8_t mode : 3;       // FIFO mode (LSM303C_FIFO_MODE)
    };
} lsm303c_fifo_config_t;

///////////////////////////////////////////////////////////////////////////////
// FIFO_SRC

/**
 * Status for FIFO_SRC.
 * Includes:
 *  - FIFO level
 *  - FIFO state: empty, full, above threshold
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t level : 5;
        uint8_t empty : 1;
        uint8_t full : 1;
        uint8_t higher_than_threshold : 1;
    };
} lsm303c_fifo_status_t;

///////////////////////////////////////////////////////////////////////////////
// IG_CFGn_A

enum LSM303C_INT_COMBINATION { LSM303C_INT_AND = 0, LSM303C_INT_OR = 1 };

/**
 * Configuration for an interrupt generator.
 * Includes:
 *  - And/or combination of events
 *  - 6D direction detection enable
 *  - Interrupt enable for per-axis thresholds
 *
 * Interrupt generator 1 has separate thresholds per axis, compared to a single
 * threshold for all axes as used by interrupt generator 2.
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t x_low_interrupt_enabled : 1;
        uint8_t x_high_interrupt_enabled : 1;
        uint8_t y_low_interrupt_enabled : 1;
        uint8_t y_high_interrupt_enabled : 1;
        uint8_t z_low_interrupt_enabled : 1;
        uint8_t z_high_interrupt_enabled : 1;
        uint8_t detect_6d_enabled : 1;
        uint8_t interrupt_combination_mode : 1;
    };
} lsm303c_xl_intgen_config_t;

///////////////////////////////////////////////////////////////////////////////
// IG_SRCn_A

/**
 * Status for IF_SRCn_A.
 * Includes:
 *  - Interrupt status
 *  - Indication of which interrupt(s) have been triggered
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t x_low : 1;
        uint8_t x_high : 1;
        uint8_t y_low : 1;
        uint8_t y_high : 1;
        uint8_t z_low : 1;
        uint8_t z_high : 1;
        uint8_t interrupt_active : 1;
    };
} lsm303c_xl_intgen_status_t;

///////////////////////////////////////////////////////////////////////////////
// IG_THSn_A

/**
 * Threshold for interrupt generator.
 * Interrupts can trigger if the recorded signal is higher or lower than the threshold, depending on configuration.
 * Thresholds are only 8-bit, and thus are only affected by reasonably significant changes.
 *
 * The specific type was needed just to simplify read/write calls.
 */
typedef union {
    uint8_t threshold;
} lsm303c_xl_intgen_threshold_config_t;

///////////////////////////////////////////////////////////////////////////////
// IG_DURn_A

typedef union {
    uint8_t raw;
    struct {
        uint8_t duration : 7;
        uint8_t duration_wait_enabled : 1;
    };
} lsm303c_xl_intgen_duration_config_t;

///////////////////////////////////////////////////////////////////////////////
// * Magnetometer definitions
///////////////////////////////////////////////////////////////////////////////
// CTRL_REG1_M

/**
 * Operative performance of the magnetometer.
 * Not entirely sure what the difference between modes actually is.
 * Datasheet tells me nothing.
 */
enum LSM303C_MAG_OPERATING_PERFORMANCE_MODE {
    LSM303C_MAG_LOW_PERFORMANCE = 0,  // Low power mode.
    LSM303C_MAG_MEDIUM_PERFORMANCE = 1,
    LSM303C_MAG_HIGH_PERFORMANCE = 2,
    LSM303C_MAG_ULTRA_HIGH_PERFORMANCE = 3
};

/**
 * Sample rate of the magnetometer.
 * The lowest sample rate is 0.625 Hz.
 */
enum LSM303C_MAG_ODR {
    LSM303C_MAG_ODR_0HZ = 0,
    LSM303C_MAG_ODR_1HZ = 1,
    LSM303C_MAG_ODR_2HZ = 2,
    LSM303C_MAG_ODR_5HZ = 3,
    LSM303C_MAG_ODR_10HZ = 4,
    LSM303C_MAG_ODR_20HZ = 5,
    LSM303C_MAG_ODR_40HZ = 6,
    LSM303C_MAG_ODR_80HZ = 7
};

/**
 * Configuration for CTRL_REG1_M.
 * Includes:
 *  - Temperature enable
 *  - Magnetometer data rate and operating mode
 *  - Self-test enable
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t self_test_enabled : 1;
        uint8_t _reserved : 1;
        uint8_t data_rate : 3;
        uint8_t operating_performance_mode : 2;
        uint8_t temperature_enabled : 1;
    };
} lsm303c_mag_temp_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG2_M

/**
 * Full-scale range of the magnetometer.
 * Only 16 gauss is available, so that's one fewer decision to make.
 */
enum LSM303C_MAG_SCALE { LSM303C_MAG_SCALE_16B = 3 };

/**
 * Configuration for CTRL_REG2_M.
 * Includes:
 *  - Reset configuration trigger
 *  - Reset memory trigger
 *  - Magnetometer scale
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved1 : 2;
        uint8_t reset_config_enabled : 1;
        uint8_t reset_memory_enabled : 1;
        uint8_t _reserved2 : 1;
        uint8_t mag_scale : 2;
        uint8_t _reserved3 : 1;
    };
} lsm303c_mag_scale_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG3_M

/**
 * Conversion mode of the magnetometer.
 * The data sheet is not clear as to whether single-conversion mode affects the power state of the magnetometer.
 * Presumably, single-conversion mode will cause the sensor to take a single reading, then put the sensor into a
 *  low-power state. However, the sensor may just not update the output register.
 *
 * Continuous conversion mode just updates the output register at the rate specified in CTRL_REG1_M.
 */
enum LSM303C_MAG_CONVERSION_MODE { LSM303C_MAG_CONTINUOUS_CONVERSION = 0, LSM303C_MAG_SINGLE_CONVERSION = 1, LSM303C_MAG_POWER_DOWN = 2 };

/**
 * Configuration for CTRL_REG3_M.
 * Includes:
 *  - Conversion mode for the magnetometer
 *  - Communication options for I2C/SPI
 *  - Low-power mode enable
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t conversion_mode : 2;   // Conversion mode of the magnetometer
        uint8_t spi_read_enabled : 1;  // Enable reads over SPI
        uint8_t _reserved1 : 2;
        uint8_t low_power_mode : 1;  // Low power mode sets the sample rate to 0.625 Hz and performs minimal filtering
        uint8_t _reserved2 : 1;
        uint8_t i2c_disabled : 1;  // Disables I2C communications
    };
} lsm303c_mag_mode_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG4_M

/**
 * Configuration for CTRL_REG4_M.
 * Includes:
 *  - Operating mode for Z axis magnetometer
 *  - Big/little endian data output
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved : 1;
        uint8_t big_endian_enabled : 1;  // 0: Big endian. LSB at low address
        uint8_t z_operating_mode : 2;    // LSM303_MAG_OPERATING_MODE
    };
} lsm303c_mag_data_config_t;

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG5_M

/**
 * Configuration for CTRL_REG5_M.
 * Includes:
 *  - Block data update enable
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved : 6;
        uint8_t block_data_update_enabled : 1;
    };
} lsm303c_mag_bdu_config_t;

///////////////////////////////////////////////////////////////////////////////
// STATUS_REG_M

/**
 * Status for STATUS_REG_M
 * Includes:
 *  - Data ready signal per axis
 *  - Data overrun indication per axis
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t data_available_x : 1;
        uint8_t data_available_y : 1;
        uint8_t data_available_z : 1;
        uint8_t data_available_all : 1;
        uint8_t data_overrun_x : 1;
        uint8_t data_overrun_y : 1;
        uint8_t data_overrun_z : 1;
        uint8_t data_overrun_any : 1;
    };
} lsm303c_mag_status_t;

///////////////////////////////////////////////////////////////////////////////
// OUT_x_x_M Registers

/**
 * Data output type for magnetometer.
 * Output is in a raw ADC format, which is 0.58 milligauss per level.
 * Temperature output is 0.125 deg C per level.
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303c_mag_raw_data_t;

/**
 * Data output type for magnetometer.
 * Magnetometer output is in milli-gauss.
 * Temperature output in deg C.
 */
typedef struct {
    float x;
    float y;
    float z;
} lsm303c_mag_corrected_data_t;

/**
 * Data output type for the magnetometer.
 * Output is the current heading in degrees with respect to magnetic North.
 * This heading may or may not be tilt corrected, depending on the read options used.
 * Temperature is included too.
 */
typedef struct {
    float heading;
} lsm303c_mag_heading_data_t;

typedef struct {
    int16_t raw;
    float temperature;
} lsm303c_temp_data_t;

///////////////////////////////////////////////////////////////////////////////
// INT_CFG_M

/**
 * Configuration for INT_CFG_M.
 * Includes:
 *  - Interrupt enable per axis
 *  - Interrupt latch
 *  - Interrupt output level
 *  - Sensor enable (weird default flag that needs to be set)
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t interrupt_enabled : 1;
        uint8_t interrupt_latch_disabled : 1;
        uint8_t interrupt_active_high_enabled : 1;
        uint8_t mag_enabled : 1;
        uint8_t _reserved : 1;
        uint8_t z_interrupt_enabled : 1;
        uint8_t y_interrupt_enabled : 1;
        uint8_t x_interrupt_enabled : 1;
    };
} lsm303c_mag_interrupt_config_t;

///////////////////////////////////////////////////////////////////////////////
// INT_SRC_M

/**
 * Status from INT_SRC_M.
 * Includes:
 *  - Per axis interrupt event indication
 *  - Interrupt status
 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t interrupt_triggered : 1;
        uint8_t range_overflow : 1;
        uint8_t z_exceeds_negative_threshold : 1;
        uint8_t y_exceeds_negative_threshold : 1;
        uint8_t x_exceeds_negative_threshold : 1;
        uint8_t z_exceeds_positive_threshold : 1;
        uint8_t y_exceeds_positive_threshold : 1;
        uint8_t x_exceeds_positive_threshold : 1;
    };
} lsm303c_mag_interrupt_status_t;

///////////////////////////////////////////////////////////////////////////////
// * LSM303C Class
///////////////////////////////////////////////////////////////////////////////

class LSM303C {
   public:
    // Start communication with the device
    bool begin(uint8_t comms_mode = LSM303C_I2C_MODE, uint8_t xl_cs_pin = 0xFF, uint8_t mag_cs_pin = 0xFF);

    // Check that communications are working by reading from the device
    bool comms_check();

    // Write configuration to a register
    void write(lsm303c_xl_inactive_threshold_t config);
    void write(lsm303c_xl_inactive_duration_t config);
    void write(lsm303c_xl_rate_config_t config);
    void write(lsm303c_xl_filter_config_t config);
    void write(lsm303c_xl_interrupt_config_t config);
    void write(lsm303c_xl_scale_bw_config_t config);
    void write(lsm303c_xl_int_dec_debug_config_t config);
    void write(lsm303c_xl_reboot_config_t config);
    void write(lsm303c_xl_int_mode_config_t config);
    void write(lsm303c_fifo_config_t config);
    void write(lsm303c_mag_temp_config_t config);
    void write(lsm303c_mag_scale_config_t config);
    void write(lsm303c_mag_mode_config_t config);
    void write(lsm303c_mag_data_config_t config);
    void write(lsm303c_mag_bdu_config_t config);
    void write(lsm303c_mag_interrupt_config_t config);
    void write(lsm303c_xl_intgen_config_t config, uint8_t int_number = 1);
    void write(lsm303c_xl_intgen_duration_config_t config, uint8_t int_number = 1);
    void write(lsm303c_xl_intgen_threshold_config_t config, uint8_t int_number = 1, char axis = 'x');

    // Read configuration from a register
    void read(lsm303c_xl_inactive_threshold_t& config);
    void read(lsm303c_xl_inactive_duration_t& config);
    void read(lsm303c_xl_rate_config_t& config);
    void read(lsm303c_xl_filter_config_t& config);
    void read(lsm303c_xl_interrupt_config_t& config);
    void read(lsm303c_xl_scale_bw_config_t& config);
    void read(lsm303c_xl_int_dec_debug_config_t& config);
    void read(lsm303c_xl_reboot_config_t& config);
    void read(lsm303c_xl_int_mode_config_t& config);
    void read(lsm303c_fifo_config_t& config);
    void read(lsm303c_mag_temp_config_t& config);
    void read(lsm303c_mag_scale_config_t& config);
    void read(lsm303c_mag_mode_config_t& config);
    void read(lsm303c_mag_data_config_t& config);
    void read(lsm303c_mag_bdu_config_t& config);
    void read(lsm303c_mag_interrupt_config_t& config);
    void read(lsm303c_xl_intgen_config_t& config, uint8_t int_number = 1);
    void read(lsm303c_xl_intgen_duration_config_t& config, uint8_t int_number = 1);
    void read(lsm303c_xl_intgen_threshold_config_t& config, uint8_t int_number = 1, char axis = 'x');

    // Read data from the sensor's output registers
    void read(lsm303c_xl_raw_data_t& data);
    void read(lsm303c_xl_corrected_data_t& data);
    void read(lsm303c_xl_roll_pitch_data_t& data);
    void read(lsm303c_mag_raw_data_t& data);
    void read(lsm303c_mag_corrected_data_t& data);
    void read(lsm303c_orientation_data_t& data);
    void read(lsm303c_temp_data_t& data);

    float get_heading(bool tilt_correct_enabled = false);
    float get_temperature();

    // Read in data from the FIFO buffer
    uint8_t read(lsm303c_xl_raw_data_t data[], uint8_t max_measurements = LSM303C_FIFO_CAPACITY);

    // Read the status of the sensor's functions
    void read(lsm303c_xl_status_t& status);
    void read(lsm303c_fifo_status_t& status);
    void read(lsm303c_mag_status_t& status);
    void read(lsm303c_mag_interrupt_status_t& status);
    void read(lsm303c_xl_intgen_status_t& status, uint8_t int_number = 1);

    void set_declination(float declination);

   private:
    typedef enum LSM303C_REGISTER {
        // XL registers
        WHO_AM_I = 0x0F,      // Accelerometer ID
        ACT_THS = 0x1E,       // Activity threshold
        ACT_DUR = 0x1F,       // Activity duration
        CTRL_REG1 = 0x20,     // Data rates and per-axis enable
        CTRL_REG2 = 0x21,     // Highpass filter configuration
        CTRL_REG3 = 0x22,     // Interrupt control and trigger status
        CTRL_REG4 = 0x23,     // Scale and AA bandwidth configuration
        CTRL_REG5 = 0x24,     // Debug and decimation
        CTRL_REG6 = 0x25,     // Force reboot register
        CTRL_REG7 = 0x26,     // Interrupt config
        STATUS_REG = 0x27,    // Accelerometer status
        OUT_X_L = 0x28,       // Axis X measurement (low)
        OUT_X_H = 0x29,       // Axis X measurement (high)
        OUT_Y_L = 0x2A,       // Axis Y measurement (low)
        OUT_Y_H = 0x2B,       // Axis Y measurement (high)
        OUT_Z_L = 0x2C,       // Axis Z measurement (low)
        OUT_Z_H = 0x2D,       // Axis Z measurement (high)
        FIFO_CTRL = 0x2E,     // Accelerometer FIFO control
        FIFO_SRC = 0x2F,      //
        IG_CFG1 = 0x30,       // XL interrupt generator 1 config
        IG_SRC1 = 0x31,       // XL interrupt generator 1 status
        IG_THS_X1 = 0x32,     // XL interrupt generator 1 X threshold
        IG_THS_Y1 = 0x33,     // XL interrupt generator 1 Y threshold
        IG_THS_Z1 = 0x34,     // XL interrupt generator 1 Z threshold
        IG_DUR1 = 0x35,       // XL interrupt generator 1 duration
        IG_CFG2 = 0x36,       // XL interrupt generator 2 config
        IG_SRC2 = 0x37,       // XL interrupt generator 2 status
        IG_THS2 = 0x38,       // XL interrupt generator 2 threshold
        IG_DUR2 = 0x39,       // XL interrupt generator 2 duration
        XL_REFERENCE = 0x3A,  // X reference (low)
        XH_REFERENCE = 0x3B,  // X reference (high)
        YL_REFERENCE = 0x3C,  // Y reference (low)
        YH_REFERENCE = 0x3D,  // Y reference (high)
        ZL_REFERENCE = 0x3E,  // Z reference (low)
        ZH_REFERENCE = 0x3F,  // Z reference (high)

        // Mag-specific registers
        TEMP_L = 0x2E,     // Temperature output
        TEMP_H = 0x2F,     // Temperature output
        INT_CFG = 0x30,    // Magnetic interrupt generator configuration
        INT_SRC = 0x31,    // Magnetic interrupt generator status
        INT_THS_L = 0x32,  // Magnetic interrupt generator threshold (low)
        INT_THS_H = 0x33   // Magnetic interrupt generator threshold (high)
    } lsm303c_reg_t;

    uint8_t _comms_mode;
    uint8_t _xl_chip_select_pin;
    uint8_t _mag_chip_select_pin;
    float _declination;

    bool write(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);
    bool read(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);

    bool write_i2c(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);
    bool read_i2c(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);

    bool write_spi(uint8_t* input, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);
    bool read_spi(uint8_t* output, lsm303c_device_address_t device_address, lsm303c_reg_t address, uint8_t length = 1);
};
#endif

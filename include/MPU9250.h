#ifndef MPU9250_H_
#define MPU9250_H_

#include <Wire.h>

/*
 * MPU9250 Main Class
 * Doesn't support SPI yet
 */
class MPU9250
{

public: // Registers
    // Gyroscope Self-Test Registers
    static const uint8_t SELF_TEST_X_GYRO = 0x00;
    static const uint8_t SELF_TEST_Y_GYRO = 0x01;
    static const uint8_t SELF_TEST_Z_GYRO = 0x02;

    // Accelerometer Self-Test Registers
    static const uint8_t SELF_TEST_X_ACCEL = 0x0D;
    static const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
    static const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

    // Gyroscope Offset Registers
    static const uint8_t XG_OFFSET_H = 0x13;
    static const uint8_t XG_OFFSET_L = 0x14;
    static const uint8_t YG_OFFSET_H = 0x15;
    static const uint8_t YG_OFFSET_L = 0x16;
    static const uint8_t ZG_OFFSET_H = 0x17;
    static const uint8_t ZG_OFFSET_L = 0x18;

    // Accelerometer Offset Registers
    static const uint8_t XA_OFFSET_H = 0x77;
    static const uint8_t XA_OFFSET_L = 0x78;
    static const uint8_t YA_OFFSET_H = 0x7A;
    static const uint8_t YA_OFFSET_L = 0x7B;
    static const uint8_t ZA_OFFSET_H = 0x7D;
    static const uint8_t ZA_OFFSET_L = 0x7E;

    // Sample Rate Divider
    static const uint8_t SMPLRT_DIV = 0x19;

    // Configuration Registers
    static const uint8_t CONFIG = 0x1A;
    static const uint8_t GYRO_CONFIG = 0x1B;
    static const uint8_t ACCEL_CONFIG = 0x1C;
    static const uint8_t ACCEL_CONFIG2 = 0x1D;
    static const uint8_t LP_ACCEL_ODR = 0x1E;
    static const uint8_t FIFO_EN = 0x23;
    static const uint8_t USER_CTRL = 0x6A;

    // I2C Slave
    uint8_t I2C_SLV_ADDR[5] = {0x25, 0x28, 0x2B, 0x2E, 0x31};
    uint8_t I2C_SLV_REG[5] = {0x26, 0x29, 0x2C, 0x2F, 0x32};
    uint8_t I2C_SLV_CTRL[5] = {0x27, 0x2A, 0x2D, 0x30, 0x34};
    uint8_t I2C_SLV_DO[5] = {0x63, 0x64, 0x65, 0x66, 0x33};
    static const uint8_t I2C_SLV4_DI = 0x35;

    // I2C Master
    static const uint8_t I2C_MST_CTRL = 0x24;
    static const uint8_t I2C_MST_STATUS = 0x36;
    static const uint8_t I2C_MST_DELAY_CTRL = 0x67;

    // Interrupt Enable
    static const uint8_t INT_PIN_CFG = 0x37;
    static const uint8_t INT_ENABLE = 0x38;
    static const uint8_t INT_STATUS = 0x3A;

    // Sensor Data Registers
    static const uint8_t ACCEL_XOUT_H = 0x3B;
    static const uint8_t ACCEL_XOUT_L = 0x3C;
    static const uint8_t ACCEL_YOUT_H = 0x3D;
    static const uint8_t ACCEL_YOUT_L = 0x3E;
    static const uint8_t ACCEL_ZOUT_H = 0x3F;
    static const uint8_t ACCEL_ZOUT_L = 0x40;

    static const uint8_t TEMP_OUT_H = 0x41;
    static const uint8_t TEMP_OUT_L = 0x42;

    static const uint8_t GYRO_XOUT_H = 0x43;
    static const uint8_t GYRO_XOUT_L = 0x44;
    static const uint8_t GYRO_YOUT_H = 0x45;
    static const uint8_t GYRO_YOUT_L = 0x46;
    static const uint8_t GYRO_ZOUT_H = 0x47;
    static const uint8_t GYRO_ZOUT_L = 0x48;

    // External Sensor Data Registers
    static const uint8_t EXT_SENS_DATA = 0x49;

    // Signal Path Reset
    static const uint8_t SIGNAL_PATH_RESET = 0x68;

    // Wake-On-Motion
    static const uint8_t WOM_THR = 0x1F;
    static const uint8_t MOT_DETECT_CTRL = 0x69;

    // Power Management
    static const uint8_t PWR_MGMT_1 = 0x6B;
    static const uint8_t PWR_MGMT_2 = 0x6C;

    // FIFO
    static const uint8_t FIFO_COUNTH = 0x72;
    static const uint8_t FIFO_COUNTL = 0x73;

    // WHO_AM_I
    static const uint8_t WHO_AM_I = 0x75;

public: // Types
    struct RawData
    {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t mag[3];
        int16_t temp;
    };

    struct CalibrationData
    {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t mag[3];
    };

    struct OffsetData
    {
        int16_t accel[3];
        int16_t gyro[3];
    };

    struct FloatData
    {
        float accel[3];
        float gyro[3];
        float mag[3];
        float temp;
    };

    struct FIFOSettings
    {
        bool temp_enable;
        bool gyro_enable;
        bool accel_enable;
        bool slv0_enable;
        bool slv1_enable;
        bool slv2_enable;
    };

    struct SlaveSettings
    {
        bool enable = true; // Enable I2C Slave
        bool byte_swap = false; // Swap bytes, but I don't know what this does
        bool reg_dis = false; // When set, the transaction does not write a register value. Only reads or writes.
        bool group_data = false; // When set, the group of data will be ends with even byte.
        uint8_t length = 0; // Length of the data to be read.
    };

    enum class GyroRange : uint8_t
    {
        RANGE_250DPS = 0b00000000,
        RANGE_500DPS = 0b00001000,
        RANGE_1000DPS = 0b00010000,
        RANGE_2000DPS = 0b00011000
    };

    enum class AccelRange : uint8_t
    {
        RANGE_2G = 0b00000000,
        RANGE_4G = 0b00001000,
        RANGE_8G = 0b00010000,
        RANGE_16G = 0b00011000
    };

    enum class DLPFMode : uint8_t
    {
        DLPF_Dec1 =     0b00000000,
        DLPF_218HZ_0 =  0b00001000,
        DLPF_218HZ_1 =  0b00001001,
        DLPF_99HZ =     0b00001010,
        DLPF_44HZ =     0b00001011,
        DLPF_21HZ =     0b00001100,
        DLPF_10HZ =     0b00001101,
        DLPF_5HZ =      0b00001110,
        DLPF_Dec2 =     0b00001111
    };

    enum class ODRControlMode : uint8_t
    {
        ODR_0_24HZ =    0b00000000,
        ODR_0_49HZ =    0b00000001,
        ODR_0_98HZ =    0b00000010,
        ODR_1_95HZ =    0b00000011,
        ODR_3_91HZ =    0b00000100,
        ODR_7_81HZ =    0b00000101,
        ODR_15_63HZ =   0b00000110,
        ODR_31_25HZ =   0b00000111,
        ODR_62_5HZ =    0b00001000,
        ODR_125HZ =     0b00001001,
        ODR_250HZ =     0b00001010,
        ODR_500HZ =     0b00001011,
    };

public: // Functions
    // MPU9250 constructor, default i2c address is 0x68
    // if you want to use a different address, call the constructor with the address
    // i2c is the TwoWire object to use, default is Wire
    MPU9250(uint8_t addr = 0x68, uint8_t ak8963_addr = 0x0C, TwoWire *i2c = &Wire);

    // Initialize the MPU9250
    // Returns true if successful
    bool setup();
    
    // Read the raw data from the MPU9250
    RawData read_raw();
    // Read the float data from the MPU9250
    FloatData read_float();

    // Write the offset data to the MPU9250
    void write_offset(const OffsetData &offset);
    // Read the offset data from the MPU9250
    OffsetData read_offset();

    // Wet sample rate divider
    void set_sample_rate_div(uint8_t div);

    // Set the wake-on-motion threshold (in mg/LSB, LSB = 4)
    void set_wom_threshold(uint8_t threshold);

    // Set the gyroscope range
    void set_gyro_range(GyroRange range);
    // Get the gyroscope range
    GyroRange get_gyro_range();

    // Set the accelerometer range
    void set_accel_range(AccelRange range);
    // Get the accelerometer range
    AccelRange get_accel_range();

    // Set the accelerometer digital low-pass filter
    void set_accel_dlpf_filter(DLPFMode mode);
    
    // Set the low power accel output data rate
    void set_accel_odr(ODRControlMode mode);

    // Set the FIFO enabled settings
    void set_fifo_enabled(const FIFOSettings &settings);

    // Set the Slave
    void set_slave_addr(int slave_num, uint8_t addr, bool write);
    void set_slave_write_mode(int slave_num, bool write);
    void set_slave_register(int slave_num, uint8_t reg);
    void set_slave_settings(int slave_num, const SlaveSettings &settings);
    void write_slave_register(int slave_num, uint8_t reg, uint8_t data);
    void read_slave_register(int slave_num, uint8_t reg, uint8_t count, uint8_t *data);

public: // Varables
    float _accel_scale = 2.0f / 32768.0f;
    float _gyro_scale = 250.0f / 32768.0f;
    float _mag_scale[3] = {0.0f, 0.0f, 0.0f};

private:
    TwoWire *_i2c;
    uint8_t _addr;
    uint8_t _ak8963_addr;

    GyroRange _gyro_range = GyroRange::RANGE_250DPS;
    AccelRange _accel_range = AccelRange::RANGE_2G;

    uint8_t slave_bytes[5] = {0};
    bool ak8963_present = false;
};

#endif
#include "MPU9250.h"

MPU9250::MPU9250(uint8_t addr = 0x68, uint8_t ak8963_addr = 0x0C, TwoWire *i2c = &Wire)
{
    _i2c = i2c;
    _addr = addr;
    _ak8963_addr = ak8963_addr;
}

bool MPU9250::setup()
{
    // Check connection
    _i2c->beginTransmission(_addr);
    uint8_t res = _i2c->endTransmission(true);
    if (res != 0)
        return false;

    // Check is the MPU9250 is connected
    _i2c->beginTransmission(_addr);
    res = _i2c->write(0x75);
    if (res != 1)
    {
        _i2c->endTransmission(true);
        return false;
    }
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    res = _i2c->read();
    if (res != 0x71 && res != 0x73)
    {
        return false;
    }

    set_slave_addr(0, 0x0C, false);
    uint8_t ak8963_asa[3];
    read_slave_register(0, 0x10, 3, &ak8963_asa[0]);
    _mag_scale[0] = (((float)ak8963_asa[0] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
    _mag_scale[1] = (((float)ak8963_asa[1] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
    _mag_scale[2] = (((float)ak8963_asa[2] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
    set_slave_register(0, 0x03);
    SlaveSettings settings; settings.length = 6;
    set_slave_settings(0, settings);

    return true;
}

MPU9250::RawData MPU9250::read_raw()
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_XOUT_H);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 14);
    uint8_t buf[14];
    for (int i = 0; i < 14; i++)
    {
        buf[i] = _i2c->read();
    }

    RawData data;
    data.accel[0] = (int16_t)(buf[0] << 8 | buf[1]);
    data.accel[1] = (int16_t)(buf[2] << 8 | buf[3]);
    data.accel[2] = (int16_t)(buf[4] << 8 | buf[5]);
    data.temp = (int16_t)(buf[6] << 8 | buf[7]);
    data.gyro[0] = (int16_t)(buf[8] << 8 | buf[9]);
    data.gyro[1] = (int16_t)(buf[10] << 8 | buf[11]);
    data.gyro[2] = (int16_t)(buf[12] << 8 | buf[13]);

    return data;
}

MPU9250::FloatData MPU9250::read_float()
{
    RawData raw = read_raw();

    FloatData data;
    data.accel[0] = (float)raw.accel[0] * _accel_scale;
    data.accel[1] = (float)raw.accel[1] * _accel_scale;
    data.accel[2] = (float)raw.accel[2] * _accel_scale;

    data.gyro[0] = (float)raw.gyro[0] * _gyro_scale;
    data.gyro[1] = (float)raw.gyro[1] * _gyro_scale;
    data.gyro[2] = (float)raw.gyro[2] * _gyro_scale;
    
    data.mag[0] = (float)raw.mag[0] * _mag_scale[0];
    data.mag[1] = (float)raw.mag[1] * _mag_scale[1];
    data.mag[2] = (float)raw.mag[2] * _mag_scale[2];

    return data;
}

void MPU9250::write_offset(const OffsetData &offset)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        _i2c->beginTransmission(_addr);
        _i2c->write(MPU9250::XG_OFFSET_H + i);
        _i2c->write(((uint8_t *)offset.gyro)[i]);
        _i2c->endTransmission(true);
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        _i2c->beginTransmission(_addr);
        _i2c->write(MPU9250::XA_OFFSET_H + i);
        _i2c->write(((uint8_t *)offset.accel)[i]);
        _i2c->endTransmission(true);
    }
}

MPU9250::OffsetData MPU9250::read_offset()
{
    OffsetData offset;
    for (uint8_t i = 0; i < 6; i++)
    {
        _i2c->beginTransmission(_addr);
        _i2c->write(MPU9250::XG_OFFSET_H + i);
        _i2c->endTransmission(false);
        _i2c->requestFrom(_addr, 1);
        ((uint8_t *)offset.gyro)[i] = _i2c->read();
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        _i2c->beginTransmission(_addr);
        _i2c->write(MPU9250::XA_OFFSET_H + i);
        _i2c->endTransmission(false);
        _i2c->requestFrom(_addr, 1);
        ((uint8_t *)offset.accel)[i] = _i2c->read();
    }

    return offset;
}

void MPU9250::set_sample_rate_div(uint8_t div)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::SMPLRT_DIV);
    _i2c->write(div);
    _i2c->endTransmission(true);
}

void MPU9250::set_wom_threshold(uint8_t threshold)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::WOM_THR);
    _i2c->write(threshold);
    _i2c->endTransmission(true);
}

void MPU9250::set_gyro_range(GyroRange range)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::GYRO_CONFIG);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    buf &= 0b11100111;
    buf |= (uint8_t)range;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::GYRO_CONFIG);
    _i2c->write(buf);
    _i2c->endTransmission(true);

    _gyro_range = range;
    switch (range)
    {
    case MPU9250::GyroRange::RANGE_250DPS:
        _gyro_scale = 250.0f / 32768.0f;
        break;
    case MPU9250::GyroRange::RANGE_500DPS:
        _gyro_scale = 500.0f / 32768.0f;
        break;
    case MPU9250::GyroRange::RANGE_1000DPS:
        _gyro_scale = 1000.0f / 32768.0f;
        break;
    case MPU9250::GyroRange::RANGE_2000DPS:
        _gyro_scale = 2000.0f / 32768.0f;
        break;
    }
}

MPU9250::GyroRange MPU9250::get_gyro_range()
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::GYRO_CONFIG);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    return (GyroRange)(buf & 0b00011000);
}

void MPU9250::set_accel_range(AccelRange range)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    buf &= 0b11100111;
    buf |= (uint8_t)range;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG);
    _i2c->write(buf);
    _i2c->endTransmission(true);

    _accel_range = range;
    switch (range)
    {
    case MPU9250::AccelRange::RANGE_2G:
        _accel_scale = 2.0f / 32768.0f;
        break;
    case MPU9250::AccelRange::RANGE_4G:
        _accel_scale = 4.0f / 32768.0f;
        break;
    case MPU9250::AccelRange::RANGE_8G:
        _accel_scale = 8.0f / 32768.0f;
        break;
    case MPU9250::AccelRange::RANGE_16G:
        _accel_scale = 16.0f / 32768.0f;
        break;
    }
}

MPU9250::AccelRange MPU9250::get_accel_range()
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    return (AccelRange)(buf & 0b00011000);
}

void MPU9250::set_accel_dlpf_filter(DLPFMode mode)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG2);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    buf &= 0b11110000;
    buf |= (uint8_t)mode;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG2);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::set_accel_odr(ODRControlMode mode)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::LP_ACCEL_ODR);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    buf &= 0b11110000;
    buf |= (uint8_t)mode;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::ACCEL_CONFIG);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::set_fifo_enabled(const FIFOSettings &settings)
{
    uint8_t buf = 0;
    buf |= (uint8_t)settings.temp_enable << 7;
    buf |= (uint8_t)settings.gyro_enable << 6;
    buf |= (uint8_t)settings.gyro_enable << 5;
    buf |= (uint8_t)settings.gyro_enable << 4;
    buf |= (uint8_t)settings.accel_enable << 3;
    buf |= (uint8_t)settings.slv2_enable << 2;
    buf |= (uint8_t)settings.slv1_enable << 1;
    buf |= (uint8_t)settings.slv0_enable;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::FIFO_EN);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::set_slave_addr(int slave_num, uint8_t addr, bool write)
{
    uint8_t buf = 0;
    buf |= (uint8_t)write << 7;
    buf |= (uint8_t)addr;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_ADDR[slave_num]);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::set_slave_write_mode(int slave_num, bool write)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_ADDR[slave_num]);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, 1);
    uint8_t buf = _i2c->read();

    buf &= 0b01111111;
    buf |= (uint8_t)write << 7;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_ADDR[slave_num]);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::set_slave_register(int slave_num, uint8_t reg)
{
    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_REG[slave_num]);
    _i2c->write(reg);
    _i2c->endTransmission(true);
}

void MPU9250::set_slave_settings(int slave_num, const SlaveSettings &settings)
{
    uint8_t buf = 0;
    buf |= (uint8_t)settings.enable << 7;
    buf |= (uint8_t)settings.byte_swap << 6;
    buf |= (uint8_t)settings.reg_dis << 5;
    buf |= (uint8_t)settings.group_data << 4;
    buf |= (uint8_t)settings.length;

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_CTRL[slave_num]);
    _i2c->write(buf);
    _i2c->endTransmission(true);
}

void MPU9250::write_slave_register(int slave_num, uint8_t reg, uint8_t data)
{
    set_slave_write_mode(slave_num, true);
    set_slave_register(slave_num, reg);

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::I2C_SLV_DO[slave_num]);
    _i2c->write(data);
    _i2c->endTransmission(true);

    set_slave_write_mode(slave_num, false);
}

void MPU9250::read_slave_register(int slave_num, uint8_t reg, uint8_t count, uint8_t *data)
{
    set_slave_write_mode(slave_num, false);
    set_slave_register(slave_num, reg);

    SlaveSettings settings;
    settings.length = count;
    set_slave_settings(slave_num, settings);

    uint8_t current = 0;
    for (int i = 0; i < slave_num; i++)
        current += slave_bytes[i];

    _i2c->beginTransmission(_addr);
    _i2c->write(MPU9250::EXT_SENS_DATA + current);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_addr, count);
    for (int i = 0; i < count; i++)
    {
        data[i] = _i2c->read();
    }
}
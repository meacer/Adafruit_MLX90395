/******************************************************************************
  This is a library for the MLX90395 magnetometer.

  Designed specifically to work with the MLX90395 breakout from Adafruit:

  ----> https://www.adafruit.com/products/4022

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code, please
  support Adafruit and open-source hardware by purchasing products from
  Adafruit!

  Written by Kevin Townsend/ktown for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 *****************************************************************************/

#include "Adafruit_MLX90395.h"

Adafruit_MLX90395::Adafruit_MLX90395(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */

bool Adafruit_MLX90395::begin_I2C(uint8_t i2c_addr, TwoWire *wire) {
  if (!i2c_dev) {
    i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  }
  spi_dev = NULL;

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_MLX90395::begin_SPI(uint8_t cs_pin, uint32_t frequency, SPIClass *theSPI) {
  i2c_dev = NULL;
  if (!spi_dev) {
    _cspin = cs_pin;
    spi_dev = new Adafruit_SPIDevice(cs_pin,
                                     frequency,
                                     SPI_BITORDER_MSBFIRST, // bit order
                                     SPI_MODE3,             // data mode
                                     theSPI);
  }
  if (!spi_dev->begin()) {
    return false;
  }
  return _init();
}

bool Adafruit_MLX90395::_init(void) {
  if (!exitMode())
    return false;

  delay(10);

  if (!reset())
    return false;

  delay(10);

  _gain = getGain();
  if (_gain == 8) { // default high field gain
    _uTLSB = 7.14;
  } else {
    _uTLSB = 2.5; // medium field gain
  }

  _resolution = getResolution();

  if (!readRegister(0x26, &uniqueID[0]) || !readRegister(0x27, &uniqueID[1]) ||
      !readRegister(0x28, &uniqueID[2])) {
    return false;
  }

  return true;
}

/**
 * Performs a single X/Y/Z conversion and returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::readData(float *x, float *y, float *z) {
  if (!startSingleMeasurement())
    return false;
  while (!readMeasurement(x, y, z))
    delay(1);

  return true;
}

/**
 * Reads data from data register & returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True on command success
 */
bool Adafruit_MLX90395::readMeasurement(float *x, float *y, float *z) {
  uint8_t rx[12] = {0};   // status, crc, X16, Y16, Z16, T16, V16

  /* Perform the transaction. */
  if (i2c_dev) {
    uint8_t tx[1] = {0x80}; // Read memory command
    if (!i2c_dev->write_then_read(tx, 1, rx, 12)) {
      return false;
    }    
    // check status
    // Serial.print("Status: "); Serial.println(rx[0], HEX);
    if (rx[0] != MLX90395_STATUS_DRDY) {
      Serial.print("I2C readMeasurement MLX90395_STATUS_DRDY failed, rx[0]: ");
      Serial.println(rx[0]);
      return false;
    }
  } else {
    // SPI RM command (0x40) is different than the I2C command (0x80).
    uint8_t tx[1] = {MLX90395_REG_RM};
    if (!spi_dev->write_then_read(tx, sizeof(tx), rx, sizeof(rx), 0x00)) {
      return false;
    }
    // check status
    // Serial.print("Status: "); Serial.println(rx[0], HEX);
    // if ((rx[0] & 1) != MLX90395_STATUS_DRDY) {
    //   Serial.print("SPI readMeasurement MLX90395_STATUS_DRDY failed, rx[0]: ");
    //   Serial.println(rx[0]);
    //   return false;
    // }
  }

  int16_t xi, yi, zi;

  // Convert data to uT and float.
  xi = (rx[2] << 8) | rx[3];
  yi = (rx[4] << 8) | rx[5];
  zi = (rx[6] << 8) | rx[7];

  *x = xi;
  *y = yi;
  *z = zi;

  // multiply by gain & LSB
  *x *= gainMultipliers[_gain] * _uTLSB;
  *y *= gainMultipliers[_gain] * _uTLSB;
  *z *= gainMultipliers[_gain] * _uTLSB;

  return true;
}

/**
 * Perform a soft reset
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::reset(void) {
  if (i2c_dev) {
    return (command(MLX90395_REG_RT) == MLX90395_STATUS_RESET);
  }
  
  // RT does not return a status byte (page 22 of the datasheet revision 6.0),
  // so just check if the write succeeded.
  // TODO: We could probably use
  // `transceive(tx, sizeof(tx), NULL, 0, 5) != MLX90395_STATUS_OK` here.
  uint8_t tx[1] = {MLX90395_REG_RT};
  return spi_dev->write_then_read(tx, sizeof(tx), NULL, 0, 0x00);
}

/**
 * Perform a mode exit
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::exitMode(void) {
  // do once and ignore status
  if (i2c_dev) {
    command(MLX90395_REG_EX);
    // TODO: Return command == MLX90395_STATUS_OK?
    return true;
  } 

  uint8_t tx[1] = {MLX90395_REG_EX};
  /* Perform the transaction. */
  transceive(tx, sizeof(tx), NULL, 0, 0);
  /* Perform the transaction again. */
  return transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90395_STATUS_OK;
}

/**
 * Begin a single measurement on all axes
 *
 * @return True on command success
 */
bool Adafruit_MLX90395::startSingleMeasurement(void) {
  if (i2c_dev) {
    return (command(MLX90395_REG_SM | MLX90395_AXIS_ALL) == MLX90395_STATUS_SMMODE);
  }
  
  /* Set the device to single measurement mode */
  uint8_t tx[1] = {MLX90395_REG_SM | MLX90395_AXIS_ALL};
  uint8_t stat = transceive(tx, sizeof(tx), NULL, 0, 0);
  if ((stat == MLX90395_STATUS_OK) || (stat == MLX90395_STATUS_SMMODE)) {
    return true;
  }
  return false;
}

bool Adafruit_MLX90395::startBurstMeasurement(uint8_t axis) {
  uint8_t tx[1] = {MLX90395_REG_SB | axis};

  /* Set the device to burst measurement mode */
  uint8_t stat = transceive(tx, sizeof(tx), NULL, 0, 0);
  if ((stat == MLX90395_STATUS_OK) || (stat == MLX90395_STATUS_SMMODE)) {
    return true;
  }
  return false;
}

/**
 * Get the current oversampling setting
 * @return MLX90395_OSR_1, MLX90395_OSR_2, MLX90395_OSR_4, or MLX90395_OSR_8
 */
mlx90393_osr_t Adafruit_MLX90395::getOSR(void) {
  if (i2c_dev) {
    Adafruit_BusIO_Register reg2 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits osr_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 0);
    return (mlx90393_osr_t)osr_bits.read();
  }
  // TODO: Implement for SPI.
  return MLX90395_OSR_1;
}

/**
 * Set the current oversampling setting
 * @param osrval MLX90395_OSR_1, MLX90395_OSR_2, MLX90395_OSR_4,
 *  or MLX90395_OSR_8
 * @return True on command success
 */
bool Adafruit_MLX90395::setOSR(mlx90393_osr_t osrval) {
  if (i2c_dev) {
    Adafruit_BusIO_Register reg2 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits osr_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 0);
    return osr_bits.write(osrval);
  }
  // TODO: Implement for SPI.
  return false;
}

/**
 * Get the current gainsel value, see DS 18.6. Sensitivity for values
 * @return 0-15 gain selection offset
 */
uint8_t Adafruit_MLX90395::getGain(void) {
  if (i2c_dev) {
    Adafruit_BusIO_Register reg0 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_0, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits gain_bits =
        Adafruit_BusIO_RegisterBits(&reg0, 4, 4);
    return gain_bits.read();
  } 
  // SPI:
  uint16_t gain;
  readRegisterSPI(MLX90395_REG_0, &gain);
  // mask off gain bits
  gain &= 0x0070;
  return gain >> MLX90395_GAIN_SHIFT;
}

/**
 * Get the current gainsel value, see DS 18.6. Sensitivity for values
 * @param gainval 0-15 gain selection offset
 * @return True on command success
 */
bool Adafruit_MLX90395::setGain(uint8_t gainval) {
  gainval &= 0xF;
  _gain = gainval; // cache it

  if (i2c_dev) {
    Adafruit_BusIO_Register reg0 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_0, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits gain_bits =
        Adafruit_BusIO_RegisterBits(&reg0, 4, 4);
    return gain_bits.write(gainval);
  }
  // TODO: Implement for SPI.
  return false;
}

/**
 * Get the resolution, note that its till 16 bits just offset within the 19
 * bit ADC output.
 * @return MLX90395_RES_16, MLX90395_RES_17, MLX90395_RES_18 or
 *  MLX90395_RES_19
 */
mlx90393_res_t Adafruit_MLX90395::getResolution(void) {
  if (i2c_dev) {
    Adafruit_BusIO_Register reg2 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits resX_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 5);
    return (mlx90393_res_t)resX_bits.read();
  }
  // TODO: Implement for SPI.
  return MLX90395_RES_19;
}

/**
 * Set the resolution, note that its till 16 bits just offset within the 19
 * bit ADC output.
 * @param resval MLX90395_RES_16, MLX90395_RES_17, MLX90395_RES_18 or
 *  MLX90395_RES_19
 * @return True on command success
 */
bool Adafruit_MLX90395::setResolution(mlx90393_res_t resval) {
  _resolution = resval; // cache it

  if (i2c_dev) {
    Adafruit_BusIO_Register reg2 =
        Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
    Adafruit_BusIO_RegisterBits resX_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 5);
    resX_bits.write(resval);
    Adafruit_BusIO_RegisterBits resY_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 7);
    resY_bits.write(resval);
    Adafruit_BusIO_RegisterBits resZ_bits =
        Adafruit_BusIO_RegisterBits(&reg2, 2, 9);
    return resZ_bits.write(resval);
  }
  // TODO: Implement for SPI.
  return false;
}

/****************************************************************/

uint8_t Adafruit_MLX90395::command(uint8_t cmd) {
  uint8_t tx[2] = {0x80, cmd};
  uint8_t status = 0xFF;

  /* Perform the transaction. */
  if (i2c_dev) {
    if (!i2c_dev->write_then_read(tx, 2, &status, 1)) {
      // Serial.println("Failed command");
      return 0xFF;
    }
  }
  return status;
}

bool Adafruit_MLX90395::readRegister(uint8_t reg, uint16_t *data) {
  /* Perform the transaction. */
  if (i2c_dev) {
    Adafruit_BusIO_Register regis =
        Adafruit_BusIO_Register(i2c_dev, reg << 1, 2);
    *data = regis.read();
    return true;
  }
  return readRegisterSPI(reg, data);
}

bool Adafruit_MLX90395::readRegisterSPI(uint8_t reg, uint16_t *data) {
  uint8_t tx[2] = {MLX90395_REG_RR,
                   reg << 2}; // the register itself, shift up by 2 bits!
  uint8_t rx[2];

  /* Perform the transaction. */
  if (transceive(tx, sizeof(tx), rx, sizeof(rx), 0) != MLX90395_STATUS_OK) {
    return false;
  }
  *data = ((uint16_t)rx[0] << 8) | rx[1];
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t device data, Adafruit Unified Sensor format
    @param  sensor Pointer to an Adafruit Unified sensor_t object that we'll
   fill in
*/
/**************************************************************************/
void Adafruit_MLX90395::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MLX90395", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -120000; // -120 gauss in uTesla
  sensor->max_value = 120000;  // +120 gauss in uTesla
  sensor->resolution = _uTLSB; // 240/16-bit uTesla per LSB
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  event Pointer to an Adafruit Unified sensor_event_t object that
   we'll fill in
    @returns True on successful read
*/
/**************************************************************************/
bool Adafruit_MLX90395::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = 1;
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();

  return readData(&event->magnetic.x, &event->magnetic.y, &event->magnetic.z);
}

/**
 * Performs a full read/write transaction with the sensor.
 *
 * @param txbuf     Pointer the the buffer containing the data to write.
 * @param txlen     The number of bytes to write.
 * @param rxbuf     Pointer to an appropriately large buffer where data read
 *                  back will be written.
 * @param rxlen     The number of bytes to read back (not including the
 *                  mandatory status byte that is always returned).
 *
 * @return The status byte from the IC.
 */
uint8_t Adafruit_MLX90395::transceive(uint8_t *txbuf, uint8_t txlen,
                                      uint8_t *rxbuf, uint8_t rxlen,
                                      uint8_t interdelay) {
  uint8_t status = 0;
  uint8_t i;
  uint8_t rxbuf2[rxlen + 2] = {0};

  if (i2c_dev) {
    /* Write stage */
    if (!i2c_dev->write(txbuf, txlen)) {
      return MLX90395_STATUS_ERROR;
    }
    delay(interdelay);

    /* Read status byte plus any others */
    if (!i2c_dev->read(rxbuf2, rxlen + 1)) {
      return MLX90395_STATUS_ERROR;
    }
    status = rxbuf2[0];
    for (i = 0; i < rxlen; i++) {
      rxbuf[i] = rxbuf2[i + 1];
    }
  }

  if (spi_dev) {

    spi_dev->write_then_read(txbuf, txlen, rxbuf2, rxlen + 1, 0x00);
    status = rxbuf2[0];
    // Serial.print("STATIS: ");
    // Serial.println(status);
    for (i = 0; i < rxlen; i++) {
      rxbuf[i] = rxbuf2[i + 1];
    }

    delay(interdelay);
  }

  /* Mask out bytes available in the status response. */
  return (status >> 2);
}

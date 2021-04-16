#ifndef ADAFRUIT_MLX90395_H
#define ADAFRUIT_MLX90395_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

#define MLX90395_STATUS_OK (0x00)     /**< OK value for status response. */
#define MLX90395_STATUS_RESET 0x02
#define MLX90395_STATUS_SMMODE 0x20
#define MLX90395_STATUS_DRDY 0x01
#define MLX90395_REG_0 0x0
#define MLX90395_REG_1 0x2
#define MLX90395_REG_2 0x4

#define MLX90395_STATUS_SMMODE (0x08) /**< SM Mode status response. */
// #define MLX90395_STATUS_RESET (0x01)  /**< Reset value for status response. */
#define MLX90395_STATUS_ERROR (0xFF)  /**< OK value for status response. */
#define MLX90395_STATUS_MASK (0xFC)   /**< Mask for status OK checks. */

#define MLX90395_AXIS_T (0x01)
#define MLX90395_AXIS_X (0x02)
#define MLX90395_AXIS_Y (0x04)
#define MLX90395_AXIS_Z (0x08)
#define MLX90395_AXIS_ALL (0x0F)      /**< T+X+Y+Z axis bits for commands. */

#define MLX90395_CONF1 (0x00)         /**< Gain */
#define MLX90395_CONF2 (0x01)         /**< Burst, comm mode */
#define MLX90395_CONF3 (0x02)         /**< Oversampling, filter, resolution. */
#define MLX90395_CONF4 (0x03)         /**< Sensitivty drift. */
#define MLX90395_OSR_SHIFT (0)        /**< Left-shift for OSR bits. */
#define MLX90395_GAIN_SHIFT (4)       /**< Left-shift for gain bits. */
#define MLX90395_RESOLUTION_SHIFT (5) /**< Left-shift for resolution bits. */
#define MLX90395_HALL_CONF (0x0C)     /**< Hall plate spinning rate adj. */

/** Register map. */
enum {
  MLX90395_REG_SB = (0x10),  /**< Start burst mode. */
  MLX90395_REG_SW = (0x20),  /**< Start wakeup on change mode. */
  MLX90395_REG_SM = (0x30),  /**> Start single-meas mode. */
  MLX90395_REG_RM = (0x40),  /**> Read measurement (SPI). */
  MLX90395_REG_RR = (0x50),  /**< Read register (SPI). */
  MLX90395_REG_WR = (0x60),  /**< Write register (SPI). */
  MLX90395_REG_EX = (0x80),  /**> Exit mode. */
  MLX90395_REG_RV = (0xC0),  /**> Read voltage. */
  MLX90395_REG_HR = (0xD0),  /**< Memory recall. */
  MLX90395_REG_HS = (0x70),  /**< Memory store. */
  MLX90395_REG_RT = (0xF0),  /**< Reset. */
  MLX90395_REG_NOP = (0x00), /**< NOP. */
};

typedef enum mlx90393_osr {
  MLX90395_OSR_1,
  MLX90395_OSR_2,
  MLX90395_OSR_4,
  MLX90395_OSR_8,
} mlx90393_osr_t;

typedef enum mlx90393_res {
  MLX90395_RES_16,
  MLX90395_RES_17,
  MLX90395_RES_18,
  MLX90395_RES_19,
} mlx90393_res_t;

static const float gainMultipliers[16] = {
    0.2, 0.25,  0.3333, 0.4, 0.5,  0.6, 0.75,  1,
    0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};

#define MLX90395_DEFAULT_ADDR (0x0C) /* Can also be 0x18, depending on IC */

/** Class for interfacing with MLX90395 magnetometer */
class Adafruit_MLX90395 : public Adafruit_Sensor {
public:
  Adafruit_MLX90395();
  bool begin_I2C(uint8_t i2c_addr = MLX90395_DEFAULT_ADDR,
                 TwoWire *wire = &Wire);
  bool begin_SPI(uint8_t cs_pin,
                 uint32_t frequency = 1000000,
                 SPIClass *theSPI = &SPI);

  bool reset(void);
  bool exitMode(void);
  bool startSingleMeasurement(void);
  bool startBurstMeasurement(uint8_t axis = MLX90395_AXIS_ALL);
  bool readMeasurement(float *x, float *y, float *z);
  bool readData(float *x, float *y, float *z);

  mlx90393_osr_t getOSR(void);
  bool setOSR(mlx90393_osr_t osrval);
  mlx90393_res_t getResolution(void);
  bool setResolution(mlx90393_res_t resval);
  uint8_t getGain(void);
  bool setGain(uint8_t gainval);

  void getSensor(sensor_t *sensor);
  bool getEvent(sensors_event_t *event);

  uint16_t uniqueID[3]; ///< 48 bits of unique identifier, read during init

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_SPIDevice *spi_dev = NULL;

  bool _init(void);
  uint8_t command(uint8_t cmd);
  bool readRegister(uint8_t reg, uint16_t *data);
  bool readRegisterSPI(uint8_t reg, uint16_t *data);

  bool writeRegisterSPI(uint8_t reg, uint16_t data);

  uint8_t transceive(uint8_t *txbuf, uint8_t txlen, uint8_t *rxbuf = NULL,
                     uint8_t rxlen = 0, uint8_t interdelay = 10);

  mlx90393_res_t _resolution = MLX90395_RES_17;
  uint8_t _gain = 0;
  float _uTLSB = 0;
  int32_t _sensorID = 90395;
  int _cspin;
};

#endif

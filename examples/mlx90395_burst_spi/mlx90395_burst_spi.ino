#include "Adafruit_MLX90395.h"

Adafruit_MLX90395 sensor = Adafruit_MLX90395();
#define MLX90395_CS 10

#define SPI_SPEED 5000000

unsigned long last_time;
int counter = 0;

void setup(void)
{
  Serial.begin(500000);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90395 Demo in burst mode using SPI");

  // To keep the measurements fast, set the following:
  // - A high SPI frequency
  // - Low oversampling
  // - Low resolution
  // These settings should return around 5150 measurements per
  // second on an Arduino Uno.
  if (! sensor.begin_SPI(MLX90395_CS, SPI_SPEED)) {
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  Serial.print("Found a MLX90395 sensor with unique id 0x");
  Serial.print(sensor.uniqueID[0], HEX);
  Serial.print(sensor.uniqueID[1], HEX);
  Serial.println(sensor.uniqueID[2], HEX);

  sensor.setOSR(MLX90395_OSR_1);
  Serial.print("OSR set to: ");
  switch (sensor.getOSR()) {
    case MLX90395_OSR_1: Serial.println("1 x"); break;
    case MLX90395_OSR_2: Serial.println("2 x"); break;
    case MLX90395_OSR_4: Serial.println("4 x"); break;
    case MLX90395_OSR_8: Serial.println("8 x"); break;
  }
  
  sensor.setResolution(MLX90395_RES_16);
  Serial.print("Resolution: ");
  switch (sensor.getResolution()) {
    case MLX90395_RES_16: Serial.println("16b"); break;
    case MLX90395_RES_17: Serial.println("17b"); break;
    case MLX90395_RES_18: Serial.println("18b"); break;
    case MLX90395_RES_19: Serial.println("19b"); break;
  }
  
  Serial.print("Gain: "); Serial.println(sensor.getGain());
  last_time = micros();
  sensor.startBurstMeasurement();
}

void loop(void) {
  counter++;
  float x, y, z;

  // Get X Y and Z data at once. 
  if (sensor.readMeasurement(&x, &y, &z)) {
    if (counter == 1000) {
      unsigned long now = micros();
      int fps = 1000000 * counter / (now - last_time);
      counter = 0;
      last_time = now;
      Serial.print("Measurements per second: "); Serial.print(fps);
      Serial.print(" \tX: "); Serial.print(x, 4); Serial.print(" uT");
      Serial.print(" \tY: "); Serial.print(y, 4); Serial.print(" uT");
      Serial.print(" \tZ: "); Serial.print(z, 4); Serial.println(" uT");
    }
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }


//  /* Get a new sensor event, normalized to uTesla */
//  sensors_event_t event; 
//  sensor.getEvent(&event);
//  /* Display the results (magnetic field is measured in uTesla) */
//  Serial.print("X: "); Serial.print(event.magnetic.x);
//  Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
//  Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
//  Serial.println(" uTesla ");
}

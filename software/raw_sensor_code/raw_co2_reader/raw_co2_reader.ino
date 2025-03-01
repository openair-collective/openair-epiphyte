// Simple Reader for STC3X
#include <Wire.h>
#include <SensirionI2cStc3x.h>

#ifndef SENSIRION_NO_ERR
#undef SENSIRION_NO_ERR
#endif 

#define SENSIRION_NO_ERR 0x0

// STC3X Repository Link: https://github.com/Sensirion/arduino-i2c-stc3x/tree/master
SensirionI2cStc3x seeO2_1;

// Wrapper for the STC31_C CO2 Sensor temperature and co2 readings
struct SeeO2Readings {
  float gas_concentration;
  float temperature;
  int status;

  // default constructor
  SeeO2Readings() : gas_concentration(0.0f), temperature(0.0f), status(0) {}

  // other... constructor
  SeeO2Readings(float gas_conc, float temp, int status) {
    this->gas_concentration = gas_conc;
    this->temperature = temp;
    this->status = status;
  }

  // constructor with Sensirion CO2 sensor passed in
  SeeO2Readings(SensirionI2cStc3x& seeO2_sensor) {
    float seeO2_gas_concentration;
    float seeO2_temperature;
    int err = seeO2_sensor.measureGasConcentration(seeO2_gas_concentration, seeO2_temperature);
    
    this->gas_concentration = seeO2_gas_concentration;
    this->temperature = seeO2_temperature;
    this->status = err;
  }

  // Validate the SeeO2Readings::status field to see if an error occurred or not
  SeeO2Readings& validate_status() {
    if (this->status != SENSIRION_NO_ERR) {
      Serial.println("Error encountered while calling SensirionI2cStc3x::measureGasConcentration");
      exit(this->status);
    }

    return *this;
  }

  // Print the coupled together temperature and co2 readings
  SeeO2Readings& print_readings() {
    Serial.print(this->gas_concentration);
    Serial.print(",");
    Serial.print(this->temperature);
    return *this;
  }
};

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  seeO2_1.begin(Wire, STC31_C_I2C_ADDR_29);

  int err = seeO2_1.setBinaryGas(0x0013); //STC31-C: 0x0013: CO₂ in air for range in 0 to 40 vol%
  if (err != SENSIRION_NO_ERR) {
    Serial.println("Error encountered while calling SensirionI2cStc3x::setBinaryGas");
    exit(err);
  }

  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  
  // float REFERENCE_TEMPERATURE = ???;
  // int err;
  // if (err = seeO2_1.setTemperature(REFERENCE_TEMPERATURE)) {
  //    Serial.println("Error encountered while calling SensirionI2cStc3x::setTemperature");
  //    exit(err);
  // }

  SeeO2Readings(seeO2_1).validate_status().print_readings();
  Serial.println();
}

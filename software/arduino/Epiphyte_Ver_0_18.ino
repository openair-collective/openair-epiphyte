/*********************************************************************
* Project Epiphyte DAC System Control Program
* OpenAir Collective
* David Wilson dw@ieee.org
**********************************************************************
* Version 0.6: Add structure to provide flexible number and types
*   of sensors.
* Version 0.7: Clean up output routines. 
* Version 0.8: Add Renesas FS1015 airflow sensor
* Version 0.9: Add timestamp (hours since start, in lieu of RTC)
* Version 0.9: Add PWM for controlling heater
* Version 0.10: Add switch debouncing and other features to make set/run robust
* Version 0.11: Add PWM control for second heater
* Version 0.12: Add PID temperature control for sorbent heaters
* Version 0.13: Various improvements to PID control
* Version 0.14: Replace "velocity" form of PID eqn with "positional" form
* Version 0.15: Add limits to integrated error err_int
* Version 0.16: Updates to CO2 sensor
* Version 0.17: Debug CO2 sensors
* Version 0.18: Clean up for Github
*********************************************************************/
/*********************************************************************
 * IMPORTANT:
 * This software is written for the Adafruit ESP32 Feather V2 and has
 * not been tested on any other processor. Product page:
 *              https://www.adafruit.com/product/5400
 * Be sure to follow instructions on loading drivers and adding board
 * to the Board Manager. Make sure correct board is selected in IDE.
 * Make sure all libraries are installed. One must be edited (below)
*********************************************************************/
#define VERSION "0.18"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SensirionI2cScd30.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"
#include "Adafruit_MPRLS.h"
#include <SparkFun_FS3000_Arduino_Library.h> //NOTE: must edit library header file to set I2C address:
                                             //  #define FS3000_DEVICE_ADDRESS 0x50     

// USER OPTIONS
#define SCD30_MEAS_INTERVAL_SECS  2
#define DISP_UPDATE_INTERVAL_SECS 6
#define PRINT_TIMESTAMP           1   //Set nonzero to print timestamp with data 
#define PRINT_SCD30_RAW           0   //Set nonzero to print raw SCD30 data

//Pin assignments
#define NEOPIXEL_PIN  0
#define swRUN         12  //Toggle switch 
#define swSET         13  //Toggle switch
#define BUTTON_C      14  //Not currently used
#define BUTTON_A      15  //Not currently used
#define HEAT_PWM_1    27    
#define BUTTON_B      32  //Not currently used
#define HEAT_PWM_2    33    

#define MPRLS_RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define MPRLS_EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 2 //Recommendation in app note https://esphome.io/components/output/ledc.html
#define PWM_RESOLUTION  14
#define PWM_FREQ     1000
#define PROC_CLOCK  80000000
#define MAX_PWM_DC  95  //Maximum duty cycle in percent (circuits seem to get hot if it's 100%)
#define MIN_PWM_DC  0   //Minimum duty cycle in percent

#define MAX_TEMP_C  200

//I2C Addresses
#define MUX0        0x70  //Not currently used
#define MUX1        0x71  //I2C MUX address
#define MUX2        0x72  //I2C MUX address
#define DISPADDR    0x3C
#define THERMADDR   0x67 

#define GREEN   0,255,0
#define BLUE    0,0,255
#define RED     255,0,0
#define YELLOW  127,127,0

//Constructors for libraries
SensirionI2cScd30 scd30;
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MCP9600 therm;
Adafruit_MPRLS press = Adafruit_MPRLS(MPRLS_RESET_PIN, MPRLS_EOC_PIN);
FS3000 fs;

//Main State Machine
enum State {STATE_UNK, STATE_SET, STATE_RUN};

enum sensorTypes {SCD30, THERM, PRESS, FLOW};
enum print {NOPRINT, PRINT};
enum disp {NODISP, DISP};
enum read {NOREAD, READ};

struct devices {
  uint8_t type;           //Refer to sensorTypes enums
  const char *label;      //Will appear in print or disp
  uint8_t muxAddr;        //I2C address of MUX switch
  uint8_t muxChan;        //MUX channel used
  uint8_t devAddr;        //Device I2C address; can use 0 if built-in to library
  uint8_t ifPrint;        //NOPRINT or PRINT
  uint8_t ifDisp;         //NODISP or DISP
  uint8_t ifRead;         //NOREAD or READ
};

/* Fill in this structure in correct format for each device.
*  Setting NOREAD for unused devices saves cycles and allows more frequent
*  read of other devices.
*  Note: SCD30 REQUIRES THREE CONTIGUOUS LINES!
*  Note: don't remove or comment out any of the thermistors through Th5
*  or the heater control loop won't work. This is a bug that needs to be fixed. */
struct devices dev[] = {
  THERM, "Th0", MUX2, 0, THERMADDR, NOPRINT,    NODISP,   NOREAD,
  THERM, "Th1", MUX2, 1, THERMADDR, PRINT,  NODISP,   NOREAD,
  THERM, "Th2", MUX2, 2, THERMADDR, PRINT,  NODISP,   NOREAD,
  THERM, "Th3", MUX2, 3, THERMADDR, PRINT,    DISP,   READ,
  THERM, "Th4", MUX2, 4, THERMADDR, PRINT,  NODISP, READ,
  THERM, "Th5", MUX2, 5, THERMADDR, PRINT,  DISP, READ,
  THERM, "Th6", MUX2, 6, THERMADDR, PRINT,  NODISP, NOREAD,
  THERM, "Th7", MUX2, 7, THERMADDR, PRINT,  NODISP, NOREAD,
  SCD30, "C0 ", MUX1, 3, 0, PRINT, DISP, READ,   //SCD30: Line 1 of 3
  SCD30, "T0 ", MUX1, 3, 0, PRINT, NODISP, READ,   //SCD30: Line 2 of 3
  SCD30, "H0 ", MUX1, 3, 0, PRINT, NODISP, READ,  //SCD30: LIne 3 of 3
  SCD30, "C1 ", MUX1, 4, 0, PRINT, DISP, READ,  //SCD30: Line 1 of 3
  SCD30, "T1 ", MUX1, 4, 0, PRINT, NODISP, READ,  //SCD30: Line 2 of 3
  SCD30, "H1 ", MUX1, 4, 0, PRINT, NODISP, READ,   //SCD30: LIne 3 of 3 
//  PRESS, "P0 ", MUX1, 5, 0, PRINT, NODISP, READ,
  FLOW,   "F0 ", MUX1, 2, 0, PRINT, DISP,  NOREAD,
  FLOW,   "F1 ", MUX1, 1, 0, PRINT, DISP, NOREAD, 
};

#define NDEVS (sizeof dev / sizeof (struct devices))

struct devdata {float meas; uint16_t nave; uint8_t ispresent;} data [NDEVS];

struct pidParm {
  uint8_t devIndex; //Index of sensor providing feedback from dev[] BE CAREFUL!
  uint8_t pwmIndex; //Index of PWM channel controlled by this loop
  uint16_t dT;      //Delta T in millisec
  float Kp;         //Proportional constant in % of duty cycle / temperature error in C
  float Ki;         //Integral constant in [%/(C * sec)]
  float Kd;         //Derivative constant in [(% * sec)/C]
};
struct pidParm pidParm[] = {  //Fill the structure, one line for each loop
  5, PWM_CHANNEL_1, 1000, 10.0, 1.0, 20.0, 
  3, PWM_CHANNEL_2, 1000, 10.0, 1.0, 20.0,
};

#define NLOOPS (sizeof pidParm / sizeof (struct pidParm))

struct pidData {
  uint8_t enabled;
  uint32_t lastTime;
  float setPt;      //Desired temperature in C
  float err_tk;     //Most recent error in C
  float err_tkm1;   //Previous error in C
  float err_int;    //Integral of all previous errors
  float utk;        //Calculated value for PWM duty cycle (update each time through)

} pidData[NLOOPS];

static uint8_t modeState = STATE_UNK;
uint32_t avgStartTime, sessionStartTime;
static char errorMessage[128];
static int16_t error;

uint8_t prompt;
float dutyPct1, dutyPct2;
float err, setTemp;
const uint16_t dutyMax = (uint16_t) (pow(2, PWM_RESOLUTION) - 1);

uint8_t initPWM (uint8_t chan, uint8_t pin, uint32_t freq, uint8_t resolution)
{
  if (chan > 15) {
    Serial.print("initPWM: Illegal channel number ");Serial.println(chan);
    return(1);
  }
  if (freq * dutyMax > PROC_CLOCK) {
    Serial.print("initPWM: Freq ");Serial.print(freq);Serial.println(" is too high for resolution");
    return(2);
  }
  ledcSetup(chan, freq, resolution);
  ledcAttachPin(pin, chan);
  ledcWrite(chan, 0);
  return(0);
}

void setPWM (uint8_t chan, float dutyPct)
{
  uint16_t dutyBits  = dutyPct * dutyMax / 100.0;
/*  Serial.println("Setting up PWM:");
  Serial.print(" Pct=");Serial.print(dutyPct,4);
  Serial.print(" Bits=");Serial.println(dutyBits); */
  ledcWrite(chan, dutyBits);
}

void initDevs (struct devices dev[], struct devdata data[], uint8_t n) 
{
  uint8_t nDevsActive = 0;
  Serial.print("NDEVS=");Serial.println(n);
  for (uint8_t i = 0; i < n; i++) {
    Serial.print("Device index ");Serial.print(i);Serial.print(" Type ");Serial.print(dev[i].type);Serial.print(" Label ");Serial.println(dev[i].label);    
  }
  for (uint8_t i = 0; i < n; i++) {
    Serial.print("Device index ");Serial.print(i);Serial.print(" Label ");Serial.println(dev[i].label);
    switch(dev[i].type) {
      case SCD30: {
        CO2SensorInit(dev[i], &data[i], SCD30_MEAS_INTERVAL_SECS);
        Serial.print("CO2SensorInit completed for device ");Serial.println(dev[i].label);
        data[i+2].ispresent = data[i+1].ispresent = data[i].ispresent;
        i += 2;
        break;
      }
      case THERM: {
        initTherm(dev[i], &data[i]);
        break;
      }
      case PRESS: {
        initPressure(dev[i], &data[i]);
        break;
      }
      case FLOW: {
        initFlow(dev[i], &data[i]);
        break;
      }
      default: {
        Serial.print("Device ");Serial.print(i);Serial.println(": Type not found.");
        data[i].ispresent = 0;
        break;
      }
    }
    data[i].nave = 0;
    Serial.print("Initialization completed for device ");Serial.println(dev[i].label);
  }
}

void initTherm (struct devices d, struct devdata *data) 
{       
  muxselect(d.muxAddr, d.muxChan);
  if (! therm.begin(d.devAddr)) {
    Serial.print("Thermistor ");Serial.print(d.label);Serial.println(" not found!");
    data->ispresent = 0;
  } else {
  Serial.print("Thermistor ");Serial.print(d.label);Serial.println("  found!");
  data->ispresent = 1;
  therm.setADCresolution(MCP9600_ADCRESOLUTION_18);
  therm.setThermocoupleType(MCP9600_TYPE_K);
  therm.setFilterCoefficient(3);
  therm.enable(true); 
  }
}

void initPressure (struct devices d, struct devdata *data) 
{      
  muxselect(d.muxAddr, d.muxChan);
  if (! press.begin()) {
    Serial.print("Pressure sensor ");Serial.print(d.label);Serial.println(" not found!");
    data->ispresent = 0;
  } else {
    Serial.print("Pressure sensor ");Serial.print(d.label);Serial.println("  found!"); 
    data->ispresent = 1;
  }
}

void initFlow (struct devices d, struct devdata *data) 
{      
  muxselect(d.muxAddr, d.muxChan);
  if (! fs.begin()) {
    Serial.print("Flow sensor ");Serial.print(d.label);Serial.println(" not found!");
    data->ispresent = 0;
  } else {
    Serial.print("Flow sensor ");Serial.print(d.label);Serial.println("  found!"); 
    fs.setRange(AIRFLOW_RANGE_7_MPS);
    data->ispresent = 1;
  }
}

float readTherm (struct devices d) 
{   
  muxselect(d.muxAddr, d.muxChan);
  //Serial.print(d.label);Serial.print(" Hot Junction: ");
  float temp = therm.readThermocouple();
  /*Serial.print(temp,1);
  Serial.print(" Cold Junction: ");
  Serial.println(therm.readAmbient(), 1); */
  return temp;
}

float readPressure (struct devices d)  //Reads MPRLS Pressure Sensor i, returns hPa
{     
  muxselect(d.muxAddr, d.muxChan);
  float p = press.readPressure();
  //Serial.print(d.label);Serial.print(" Pressure: ");
  //Serial.print(p,1); Serial.println(" hPa");
  return (p);
}

float readFlow (struct devices d)  //Reads Flow Sensor i, returns m/s
{     
  muxselect(d.muxAddr, d.muxChan);
  float p = fs.readMetersPerSecond();
  //Serial.print(d.label);Serial.print(" Pressure: ");
  //Serial.print(p,1); Serial.println(" hPa");
  return (p);
}

void initPID(struct pidParm p[], struct pidData d[], uint8_t n ){
  for(uint8_t i=0; i<n; i++) {
    d[i].enabled = 0;
    d[i].setPt = 25;
    d[i].err_tk = d[i].err_tkm1 = d[i].err_int = 0.0;
    d[i].utk = 0;
  }
}

float nextDC(struct pidParm p, struct pidData *d, float err) {    //Calculates next value of duty cycle for PWM loop
  d->err_tkm1 = d->err_tk;
  d->err_tk = err;
  float errIntMax = (MAX_PWM_DC - p.Kp * err) * 1000.0 / (p.Ki * p.dT);
  float errIntMin = (MIN_PWM_DC - p.Kp * err) * 1000.0 / (p.Ki * p.dT);
  d->err_int += err;
  if (d->err_int > errIntMax) {
    d->err_int = errIntMax;
  } else if (d->err_int < errIntMin) {
    d->err_int = errIntMin;
  }
  float utk =  (p.Kp * d->err_tk) + (p.Ki * d->err_int * p.dT / 1000.0) + (p.Kd * (d->err_tk - d->err_tkm1) * 1000.0 / p.dT) ;
  if (utk > (float) MAX_PWM_DC) {
    utk = MAX_PWM_DC;
  }
  if (utk < 0.0) {
    utk = 0.0;
  }
  //Serial.print(" ErrIntMin=");Serial.print(errIntMin,2);Serial.print(" ErrIntMax=");Serial.print(errIntMax,2);
  d->utk = utk;
  return utk;
}

void setPID(struct pidData *data, float t, uint8_t en ){     //Sets temperature goal and enables PWM loop i
  Serial.print("setPID: setPt="); Serial.println(t);
  data->lastTime = millis();
  data->enabled = en;
  data->setPt = t; 
}

void setup() 
{
  uint8_t ret;
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.setTimeout(10000UL);

  Serial.print("Welcome to Epiphyte! Version ");Serial.println(VERSION);

  Wire.begin();

  pixel.begin();
  pixel.clear();
  pixel.setPixelColor(0, pixel.Color(YELLOW)); 
  pixel.show();

  pinMode(swRUN, INPUT_PULLUP);
  pinMode(swSET, INPUT_PULLUP);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);


  delay(250); //Wait for OLED to power up
  display.begin(DISPADDR, true);
  display.display();
  delay(1000);
  display.setRotation(1);
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);

  Serial.print(NDEVS); Serial.println(" devices registered");
  Serial.println("Begin Initialization");

  initDevs(dev, data, NDEVS);
  Serial.println("initDevs complete.");

  ret = initPWM(PWM_CHANNEL_1, HEAT_PWM_1, PWM_FREQ, PWM_RESOLUTION);
  ret = initPWM(PWM_CHANNEL_2, HEAT_PWM_2, PWM_FREQ, PWM_RESOLUTION);

  initPID(pidParm, pidData, NLOOPS);

  sessionStartTime = avgStartTime = millis();
}

void loop() 
{
  uint8_t i;
  uint8_t swRunHi, swSetHi, swRunHiLast, swSetHiLast;
  float co2, temp, humid; 
  uint32_t sessionElapsedTime, waitForResponseStart;
  float sessionElapsedTimeHours;
  swRunHi = digitalRead(swRUN);
  swSetHi = digitalRead(swSET);
  if         (~swRunHi && swSetHi) {
              modeState = STATE_RUN;
  } else if (swRunHi  && ~swSetHi) {
              modeState = STATE_SET;
  } else {
              modeState = STATE_UNK;
  }

  switch(modeState) {
    case STATE_RUN: {
      pixel.setPixelColor(0, pixel.Color(GREEN));  
      pixel.show();
      prompt = 1;

      for (i = 0; i< NLOOPS; i++) {     //Loop for pid devices
        if ((millis() > pidData[i].lastTime + pidParm[i].dT ) && (pidData[i].enabled)) {
          float read = readTherm(dev[pidParm[i].devIndex]);
          float err = pidData[i].setPt - read;
          //Serial.print("Loop ");Serial.print(i);Serial.print(" Read=");
          //                Serial.print(read);Serial.print(" err= ");Serial.print(err,4);
          float dc = nextDC(pidParm[i], &pidData[i], err);
          //Serial.print("Loop ");Serial.print(i);Serial.print(" err_int=");Serial.print(pidData[i].err_int);Serial.print(" DC= ");Serial.println(dc,2); 
          setPWM(pidParm[i].pwmIndex, dc);
          pidData[i].lastTime = millis();
        }
      }

      for (i = 0; i < NDEVS; i++) {
        if ((data[i].ispresent) && (dev[i].ifRead)) {
          switch(dev[i].type) {
            case SCD30: {
              if (readSCD30IfReady(dev[i], co2, temp, humid, PRINT_SCD30_RAW )) {
                data[i].nave++;
                data[i+1].nave = data[i+2].nave = data[i].nave;
                data[i].meas += co2;
                data[i+1].meas += temp;
                data[i+2].meas += humid;
              } 
              i += 2;      
              break;
            }
            case THERM: {
              data[i].meas += readTherm(dev[i]);
              data[i].nave++;
              break;
            }
            case PRESS: {
              data[i].meas += readPressure(dev[i]);
              data[i].nave++;
              break;
            }
            case FLOW: {
              data[i].meas += readFlow(dev[i]);
              data[i].nave++;
              break;
            }
          }
        } 
      }
      if (millis() > avgStartTime + DISP_UPDATE_INTERVAL_SECS*1000.0) {
        sessionElapsedTime = millis() - sessionStartTime;
        sessionElapsedTimeHours = (float) sessionElapsedTime / 3600000.0;
        display.clearDisplay();
        display.setCursor(0,0);
        if (PRINT_TIMESTAMP) {Serial.print(60.0 * sessionElapsedTimeHours, 2);Serial.print(" ");}
        for (i = 0; i < NDEVS; i++) {
          if (((dev[i].ifPrint) || (dev[i].ifDisp)) && (data[i].ispresent) && (dev[i].ifRead)) {
            data[i].meas /= data[i].nave;
            if (dev[i].ifPrint) {Serial.print(dev[i].label);Serial.print("=");Serial.print(data[i].meas);Serial.print(" ");}
            if (dev[i].ifDisp) {display.print(dev[i].label);display.print("=");display.println(data[i].meas,2);}
            data[i].meas = 0.0;
            data[i].nave = 0;
          }
        }
        //Serial.print("DC=");Serial.print(pidData[0].utk);Serial.print(", ");Serial.print(pidData[1].utk);
        Serial.println();
        yield();
        display.display();
        avgStartTime = millis();
      } 
          break;
        }
        
    case STATE_SET: {
      pixel.setPixelColor(0, pixel.Color(BLUE)); 
      pixel.show();
      if (prompt) {
        Serial.println("Current Heater Temp Settings, in percent:");
        Serial.print("Heater 1: "); Serial.print(pidData[0].setPt);Serial.println("C");
        Serial.print("Heater 2: "); Serial.print(pidData[1].setPt);Serial.println("C");
        Serial.println("Enter new Heater 1 temperature: (<10 to disable)");
      }
      if (Serial.available() > 0) {
        setTemp = Serial.parseFloat();
        setTemp = (setTemp > MAX_TEMP_C) ? MAX_TEMP_C : setTemp;
        while (Serial.available() > 0) {
          Serial.read();
        }
       // setPID(pidData[0],setTemp, 1 );
        pidData[0].setPt = setTemp;
        if (setTemp < 10.0) {
          pidData[0].enabled = 0;
          setPWM(pidParm[0].pwmIndex, 0.0);
          Serial.println("Heater 1 disabled."); 
        } else {
          pidData[0].enabled = 1;
          Serial.print("Heater 1: Set temp=");Serial.println(pidData[0].setPt);
          pidData[0].utk=pidData[0].err_tk=pidData[0].err_tkm1=pidData[0].err_int=0.0;
        }
        pidData[0].lastTime = millis();


        Serial.println("Enter new Heater 2 temperature: (<10 to disable)");
        while (Serial.available() == 0) {}
        setTemp = Serial.parseFloat();
        setTemp = (setTemp > MAX_TEMP_C) ? MAX_TEMP_C : setTemp;
        while (Serial.available() > 0) {
          Serial.read();
        }
        pidData[1].setPt = setTemp;
        if (setTemp < 10.0) {
          pidData[1].enabled = 0;
          setPWM(pidParm[1].pwmIndex, 0.0);
          Serial.println("Heater 2 disabled."); 
        } else {
          pidData[1].enabled = 1;
          Serial.print("Heater 2: Set temp=");Serial.println(pidData[1].setPt);
          pidData[1].utk=pidData[1].err_tk=pidData[1].err_tkm1=pidData[1].err_int=0.0;
        }
        pidData[1].lastTime = millis();
      }
      prompt = 0;
      break;
    } 

    case STATE_UNK: {
      pixel.setPixelColor(0, pixel.Color(RED)); 
      pixel.show();
      break;
    }
  }

}

/*******************************************************************************
** Functions 
/******************************************************************************/
void muxselect(uint8_t addr, uint8_t chan) 
{
  if (chan > 7) return;
  Wire.beginTransmission(addr);
  Wire.write(1 << chan);
  Wire.endTransmission();
  delay(100); //Allow time for switch
}

/*              My functions using Sensirion's library     (MODIFIED)        
** Refer to Sensirion's copyright notice in the library source files. */
void CO2SensorInit(struct devices d, struct devdata *data, uint16_t interval) 
{
  uint8_t major = 0;
  uint8_t minor = 0;
  muxselect(d.muxAddr, d.muxChan);
  delay(100); //Gives I2C MUX time to switch over
  scd30.begin(Wire, SCD30_I2C_ADDR_61);
  scd30.stopPeriodicMeasurement();
  scd30.softReset();
  delay(2000);
  error = scd30.readFirmwareVersion(major, minor);
  if (error != NO_ERROR) {
    Serial.print("CO2 Sensor #"); Serial.print(d.label); Serial.print(": ");
    Serial.print("Error trying to execute readFirmwareVersion(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    data->ispresent = 0;
    return;
  }
  data->ispresent = 1;
  Serial.print("CO2 Sensor #"); Serial.print(d.label); Serial.print(": ");
  Serial.print("firmware version major: ");
  Serial.print(major);
  Serial.print("\t");
  Serial.print("minor: ");
  Serial.print(minor);
  Serial.println();

  error = scd30.activateAutoCalibration(0);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute activateAutoCalibration(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  } else {
    Serial.println("Auto Calibration Disabled");
  }

  error = scd30.startPeriodicMeasurement(0);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  error = scd30.setMeasurementInterval(interval);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute setMeasurementInterval(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
}

/*              Wait for Data Ready, then Read                                 */
uint8_t readSCD30IfReady(struct devices d, float& co2, float& temp, float& humid, uint8_t print) 
{
  uint16_t dataReady;
  muxselect(d.muxAddr, d.muxChan);
  //delay(100); //Gives I2C MUX time to switch over
  error = scd30.getDataReady(dataReady);
   if (error != NO_ERROR) {
    Serial.print ("CO2 Sensor# "); Serial.print(d.label); Serial.print(": ");
    Serial.print("Error trying to execute getDataReady(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return(0);
  }
  if (dataReady) {
    error = scd30.readMeasurementData(co2, temp, humid);
    if (error != NO_ERROR) {
      Serial.print ("CO2 Sensor# "); Serial.print(d.label); Serial.print(": ");
      Serial.print("Error trying to execute readMeasurementData(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return(0);
    }
    if (print) {
    Serial.print ("CO2 Sensor# "); Serial.print(d.label); Serial.print(": ");
    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temp(C): ");
    Serial.print(temp);
    Serial.print("\t");
    Serial.print("RH (%): ");
    Serial.print(humid);
    Serial.println();
    }
    return(1);
  } else {
    return (0);
  }
}


/*--------------------------------------------------*/
/*--------------- Include Libraries ----------------*/
/*--------------------------------------------------*/

#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#define U_PART U_SPIFFS

/*--------------------------------------------------*/
/*--------------- Enable multicore -----------------*/
/*--------------------------------------------------*/

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

/*--------------------------------------------------*/
/*---------------- RTOS Task List ------------------*/
/*--------------------------------------------------*/

void TaskGyroSystem( void *pvParameters );
void TaskGuidanceSystem( void *pvParameters );
void TaskPowerManagement( void *pvParameters ); 
void TaskCommSystem( void *pvParameters );

/*--------------------------------------------------*/
/*------------- I/O Pin declearation ---------------*/
/*--------------------------------------------------*/

#define PIXELPIN  18
#define GPS_RX    20
#define GPS_TX    21

#define SW_PIN  34
#define CH1_PIN 11
#define CH2_PIN 12
#define CH3_PIN 13
#define CH4_PIN 14
#define CH5_PIN 15
#define CH6_PIN 16

#define AILERON_L_PIN 1
#define AILERON_R_PIN 2
#define ELEVATOR_PIN  3
#define RUDDER_PIN    4
#define BLDC_PIN      5

#define DEBUG_BPID  1
#define DEBUG_NPID  0
#define DEBUG_BPM   0
#define DEBUG_GPS   0
#define DEBUG_RF    0

#define DEBUG_MANUAL  0
#define DEBUG_SENSOR  0

#define TRAINER 1
#define FLYWING 2
#define VTAIL   3

/*--------------------------------------------------*/
/*--------------- Define constants -----------------*/
/*--------------------------------------------------*/

// Constants for valid signal ranges
#define MIN_SIGNAL_WIDTH  1000    // Minimum signal width in microseconds
#define MAX_SIGNAL_WIDTH  2000    // Maximum signal width in microseconds

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

//flight_mode
#define AUTO   1 
#define STABLE 2
#define REMOTE 3

#define NUMPIXELS 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Drone_V0";
const char* password = "123456789";
//-----------------------------------//
const char* PARAM_LAT = "inputLatitude";
const char* PARAM_LON = "inputLongitude";
const char* PARAM_ALT = "inputAltitude";
const char* PARAM_RKP = "inputRollKp";
const char* PARAM_RKI = "inputRollKi";
const char* PARAM_RKD = "inputRollKd";
const char* PARAM_YKP = "inputYawKp";
const char* PARAM_YKI = "inputYawKi";
const char* PARAM_YKD = "inputYawKd";
const char* PARAM_PKP = "inputPitchKp";
const char* PARAM_PKI = "inputPitchKi";
const char* PARAM_PKD = "inputPitchKd";
const char* PARAM_BKP = "inputBankKp";
const char* PARAM_BKI = "inputBankKi";
const char* PARAM_BKD = "inputBankKd";
const char* PARAM_AKP = "inputAltitudeKp";
const char* PARAM_AKI = "inputAltitudeKi";
const char* PARAM_AKD = "inputAltitudeKd";
const char* PARAM_ALZ = "inputAileroLzero";
const char* PARAM_ARZ = "inputAileroRzero";
const char* PARAM_ELZ = "inputElevatorZero";
const char* PARAM_RUZ = "inputRudderZero";
const char* PARAM_ALMM = "inputAileroLMaxAngleMNL";
const char* PARAM_ARMM = "inputAileroRMaxAngleMNL";
const char* PARAM_ELMM = "inputElevatorMaxAngleMNL";
const char* PARAM_RUMM = "inputRudderMaxAngleMNL";
const char* PARAM_ALMA = "inputAileroLMaxAngleAuto";
const char* PARAM_ARMA = "inputAileroRMaxAngleAuto";
const char* PARAM_ELMA = "inputElevatorMaxAngleAuto";
const char* PARAM_RUMA = "inputRudderMaxAngleAuto";
const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>
  <title>GPS Drone Version 1.0</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    a:link, a:visited {
      background-color: #f44336;
      color: white;
      padding: 10px 10px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
    }
    a:hover, a:active {
      background-color: red;
    }
  </style>
  <script>
    function submitMessage() {
      //alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script>
</head>
<body>
<h2>GPS Drone Version 1.0</h2> <a href="/update">Firmware Update</a>
<p>
<table>
  <tr>
    <th align="left" valign="center">GPS Hdop:</th>
    <td align="left" valign="center">%gps_hdop%</td>
    <th align="left" valign="center">Ground Speed:</th>
    <td align="left" valign="center">%ground_speed% kmh</td>
    <th align="left" valign="center">Mag Heading:</th>
    <td align="left" valign="center">%mag_heading% &deg</td>
  </tr>
  <tr>
    <th align="left" valign="center">Current Latitude:</th>
    <td align="left" valign="center">%current_lat%</td>
    <th align="left" valign="center">Current Longitude:</th>
    <td align="left" valign="center">%current_lon%</td>
    <th align="left" valign="center">Current Altitude:</th>
    <td align="left" valign="center">%current_alt% m</td>
    
  </tr>
  <tr>
    <th align="left" valign="center">BPM Altitude:</th>
    <td align="left" valign="center">%bpm_altitude% m</td>
    <th align="left" valign="center">BPM Pressure:</th>
    <td align="left" valign="center">%bpm_pressure% Pa</td>
    <th align="left" valign="center">BPM Temperature:</th>
    <td align="left" valign="center">%bpm_temperature% &degC</td>
  </tr>
</table>
</p>
<p>
<table>
  <form action="/get" target="hidden-form">
  <tr>
    <th align="left" valign="center">Target Latitude:</th>
    <td align="left" valign="center">%inputLatitude%</td>
    <td align="left" valign="center">
    <input type="text" name="inputLatitude" value=%inputLatitude%  size="10">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
  </tr>
  </form>
  <form action="/get" target="hidden-form">
  <tr>
    <th align="left" valign="center">Target Longitude:</th>
    <td align="left" valign="center">%inputLongitude%</td>
    <td align="left" valign="center">
    <input type="text" name="inputLongitude" value=%inputLongitude%  size="10">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
  </tr>
  </form>
  <form action="/get" target="hidden-form">
  <tr>
    <th align="left" valign="center">Target Altitude:</th>
    <td align="left" valign="center">%inputAltitude%</td>
    <td align="left" valign="center">
    <input type="text" name="inputAltitude" value=%inputAltitude%  size="10">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
  </tr>
  </form>
</table>
</p>
<p>
<table>
  <tr>
    <th align="left" valign="center">&nbsp;</th>
    <th align="center" valign="center">Kp</th>
    <th align="center" valign="center">Ki</th>
    <th align="center" valign="center">Kd</th>
  </tr>
  
  <tr>
    <th align="left" valign="center">Roll:</th>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRollKp%
    <input type="text" name="inputRollKp" value=%inputRollKp% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRollKi%
    <input type="text" name="inputRollKi" value=%inputRollKi% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRollKd%
    <input type="text" name="inputRollKd" value=%inputRollKd% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  
  <tr>
    <th align="left" valign="center">Yaw:</th>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputYawKp%
    <input type="text" name="inputYawKp" value=%inputYawKp% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputYawKi%
    <input type="text" name="inputYawKi" value=%inputYawKi% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputYawKd%
    <input type="text" name="inputYawKd" value=%inputYawKd% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  
  <tr>
    <th align="left" valign="center">Pitch:</th>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputPitchKp%
    <input type="text" name="inputPitchKp" value=%inputPitchKp% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputPitchKi%
    <input type="text" name="inputPitchKi" value=%inputPitchKi% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputPitchKd%
    <input type="text" name="inputPitchKd" value=%inputPitchKd% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  
  <tr>
    <th align="left" valign="center">Bank:</th>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputBankKp%
    <input type="text" name="inputBankKp" value=%inputBankKp% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputBankKi%
    <input type="text" name="inputBankKi" value=%inputBankKi% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputBankKd%
    <input type="text" name="inputBankKd" value=%inputBankKd% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  
  <tr>
    <th align="left" valign="center">Altitude:</th> 
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAltitudeKp%
    <input type="text" name="inputAltitudeKp" value=%inputAltitudeKp% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAltitudeKi%
    <input type="text" name="inputAltitudeKi" value=%inputAltitudeKi% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAltitudeKd%
    <input type="text" name="inputAltitudeKd" value=%inputAltitudeKd% size="5">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
</table>
</p>
<p>
<table>
  <tr>
    <th align="left" valign="center">&nbsp;</th>
    <th align="center" valign="center">Aileron L</th>
    <th align="center" valign="center">Aileron R</th>
    <th align="center" valign="center">Elevator</th>
    <th align="center" valign="center">Rudder</th>
  </tr>
  <tr>
    <th align="left" valign="center">Zero Adjust:</th>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroLzero%
    <input type="text" name="inputAileroLzero" value=%inputAileroLzero% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroRzero%
    <input type="text" name="inputAileroRzero" value=%inputAileroRzero% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputElevatorZero%
    <input type="text" name="inputElevatorZero" value=%inputElevatorZero% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRudderZero%
    <input type="text" name="inputRudderZero" value=%inputRudderZero% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  <tr>
    <th align="left" valign="center">Max Angle (MNL):</th> 
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroLMaxAngleMNL%
    <input type="text" name="inputAileroLMaxAngleMNL" value=%inputAileroLMaxAngleMNL% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroRMaxAngleMNL%
    <input type="text" name="inputAileroRMaxAngleMNL" value=%inputAileroRMaxAngleMNL% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputElevatorMaxAngleMNL%
    <input type="text" name="inputElevatorMaxAngleMNL" value=%inputElevatorMaxAngleMNL% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRudderMaxAngleMNL%
    <input type="text" name="inputRudderMaxAngleMNL" value=%inputRudderMaxAngleMNL% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
  
  <tr>
    <th align="left" valign="center">Max Angle (Auto):</th> 
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroLMaxAngleAuto%
    <input type="text" name="inputAileroLMaxAngleAuto" value=%inputAileroLMaxAngleAuto% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputAileroRMaxAngleAuto%
    <input type="text" name="inputAileroRMaxAngleAuto" value=%inputAileroRMaxAngleAuto% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputElevatorMaxAngleAuto%
    <input type="text" name="inputElevatorMaxAngleAuto" value=%inputElevatorMaxAngleAuto% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
    <form action="/get" target="hidden-form">
    <td align="left" valign="center">
    %inputRudderMaxAngleAuto%
    <input type="text" name="inputRudderMaxAngleAuto" value=%inputRudderMaxAngleAuto% size="4">
    <input type="submit" value="Update" onclick="submitMessage()">
    </td>
    </form>
  </tr>
</table>
</p>
<iframe style="display:none" name="hidden-form"></iframe>
</body>
</html>

)rawliteral";
 
/*--------------------------------------------------*/
/*-------------- Object Declearation ---------------*/
/*--------------------------------------------------*/

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Servo aileronL, aileronR, elevator, rudder, bldc; //AileronL, AileronR, Elevator, Rudder, 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);
Adafruit_BMP280 bmp; // I2C
AsyncWebServer server(80);

/*--------------------------------------------------*/
/*------------- Variable declearation --------------*/
/*--------------------------------------------------*/

//PID
float rollKp = 10.0;
float rollKi =  0.1;
float rollKd =  5.0;

float yawKp = 10.0;
float yawKi =  0.1;
float yawKd =  5.0;

float pitchKp = 10.0;
float pitchKi =  0.1;
float pitchKd =  5.0;

float bankKp = 5.0;
float bankKi =  0.1;
float bankKd =  1.0;

float altitudeKp = 5.0;
float altitudeKi =  0.1;
float altitudeKd =  1.0;

int flight_mode = STABLE;

// Define global variables for PID controller
// target angle  
float setpoint_roll = 0.0;    
float setpoint_pitch = 0.0;
float setpoint_yaw = 0.0;

// difference between target and actual angle   
float error_roll = 0.0;        
float error_pitch = 0.0;
float error_yaw = 0.0;

// previous error value
float last_error_roll = 0.0;  
float last_error_pitch = 0.0;
float last_error_yaw = 0.0;

// sum of errors over time
float integral_roll = 0.0;    
float integral_pitch = 0.0;
float integral_yaw = 0.0;

// change in error over time
float derivative_roll = 0.0;  
float derivative_pitch = 0.0;
float derivative_yaw = 0.0;

// control variable for PID controller
float output_roll = 0.0;      
float output_pitch = 0.0;
float output_yaw = 0.0;

float setpoint_heading = 0.0;    // target angle  
float error_heading = 0.0;       // difference between target and actual angle   
float last_error_heading = 0.0;  // previous error value
float integral_heading = 0.0;    // sum of errors over time
float derivative_heading = 0.0;  // change in error over time
float output_heading = 0.0;      // control variable for PID controller

float setpoint_altitude = 0.0;    // target altitude  
float error_altitude = 0.0;       // difference between target and actual altitude   
float last_error_altitude = 0.0;  // previous error value
float integral_altitude = 0.0;    // sum of errors over time
float derivative_altitude = 0.0;  // change in error over time
float output_altitude = 0.0;      // control variable for PID controller

// Define global variables for remote
volatile uint32_t ch1_start, ch1_end, ch2_start, ch2_end, ch3_start, ch3_end, ch4_start, ch4_end, ch5_start, ch5_end, ch6_start, ch6_end;
volatile int  ch1 = 0;
volatile int  ch2 = 0;
volatile int  ch3 = 0;
volatile int  ch4 = 0;
volatile int  ch5 = 0;
volatile int  ch6 = 0;

// Gyroscope sensitivity (you may need to adjust this value for your specific sensor)
float gyro_sensitivity = 131.0; //±250
float accl_sensitivity = 16384.0; //±2g

float accel_offset_x = 0.0;  // X-axis accelerometer offset
float accel_offset_y = 0.0;  // Y-axis accelerometer offset
float accel_offset_z = 0.0;  // Z-axis accelerometer offset

float gyro_offset_x = 0.0;  // X-axis gyrometer offset
float gyro_offset_y = 0.0;  // Y-axis gyrometer offset
float gyro_offset_z = 0.0;  // Z-axis gyrometer offset

float previousYawRate = 0.0;

uint32_t time_elapsed, previous_time;

float elapsedTime, currentTime, timePrev;
float yaw, pitch, roll;
float mag_heading=0.0;

float ground_speed=0.0;
float current_lat=0.0;
float current_lon=0.0;
float current_alt=0.0; 

float gps_hdop = 0.0; //
bool gps_ready = false;

float bpm_altitude=0.0;
float bpm_pressure=0.0;
float bpm_temperature=0.0;

float target_heading=0.0;
float target_distance=0.0;

float gps_roll_angle = 0.0;
float gps_pitch_angle = 0.0;

int AILERON_L_ZERO  = 90; //AILERON_L_ZERO AILERON_R_ZERO ELEVATOR_ZERO RUDDER_ZERO
int AILERON_R_ZERO  = 90;
int ELEVATOR_ZERO   = 90;
int RUDDER_ZERO     = 90;

int AILERON_DISPLACEMENT_ML = 30;
int ELEVATOR_DISPLACEMENT_ML = 30;
int RUDDER_DISPLACEMENT_ML = 30;

int AILERON_DISPLACEMENT_PID = 30;
int ELEVATOR_DISPLACEMENT_PID = 30;
int RUDDER_DISPLACEMENT_PID = 30;

size_t content_len;

// Variables will change:
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;
bool intialState = true;
// constants won't change.
const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1500; // 1500 milliseconds

int display_page = 0;
/*--------------------------------------------------*/
/*------------ GPS Target declearation -------------*/
/*--------------------------------------------------*/
float target_lat=27.212489691630605;
float target_lon=75.70034068918126;
float target_alt=500.0;

bool armed = false;
/*--------------------------------------------------*/
/*--------------- Interrupt Routine ----------------*/
/*--------------------------------------------------*/

void IRAM_ATTR ch1_ISR() {
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1_start = micros();
  } else {
    ch1_end = micros();
    if (ch1_end - ch1_start >= MIN_SIGNAL_WIDTH && ch1_end - ch1_start <= MAX_SIGNAL_WIDTH){
      ch1 = ch1_end - ch1_start;
    }
  }
}

void IRAM_ATTR ch2_ISR() {
  if (digitalRead(CH2_PIN) == HIGH) {
    ch2_start = micros();
  } else {
    ch2_end = micros();
    if (ch2_end - ch2_start >= MIN_SIGNAL_WIDTH && ch2_end - ch2_start <= MAX_SIGNAL_WIDTH){
      ch2 = ch2_end - ch2_start;
    }
  }
}

void IRAM_ATTR ch3_ISR() {
  if (digitalRead(CH3_PIN) == HIGH) {
    ch3_start = micros();
  } else {
    ch3_end = micros();
    if (ch3_end - ch3_start >= MIN_SIGNAL_WIDTH && ch3_end - ch3_start <= MAX_SIGNAL_WIDTH){
      ch3 = ch3_end - ch3_start;
    }
  }
}

void IRAM_ATTR ch4_ISR() {
  if (digitalRead(CH4_PIN) == HIGH) {
    ch4_start = micros();
  } else {
    ch4_end = micros();
    if (ch4_end - ch4_start >= MIN_SIGNAL_WIDTH && ch4_end - ch4_start <= MAX_SIGNAL_WIDTH){
      ch4 = ch4_end - ch4_start;
    }
  }
}

void IRAM_ATTR ch5_ISR() {
  if (digitalRead(CH5_PIN) == HIGH) {
    ch5_start = micros();
  } else {
    ch5_end = micros();
    if (ch5_end - ch5_start >= MIN_SIGNAL_WIDTH && ch5_end - ch5_start <= MAX_SIGNAL_WIDTH){
      ch5 = ch5_end - ch5_start;
    }
  }
}

void IRAM_ATTR ch6_ISR() {
  if (digitalRead(CH6_PIN) == HIGH) {
    ch6_start = micros();
  } else {
    ch6_end = micros();
    if (ch6_end - ch6_start >= MIN_SIGNAL_WIDTH && ch6_end - ch6_start <= MAX_SIGNAL_WIDTH){
      ch6 = ch6_end - ch6_start;
    }
  }
}


void setup() {
  
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    Wire.begin();
    
    // Initialize SPIFFS
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
    
    initGPIO();
    
    pixel.begin();    // INITIALIZE NeoPixel
    pixel.clear();    // Set pixel colors to 'off'
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.setBrightness(10);
    pixel.show();

    //WiFi.mode(WIFI_STA);
    //WiFi.begin(ssid, password);
    //if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //  Serial.println("WiFi Failed!");
    //  return;
    //}
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    startWebServer();
    
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    
    refreshUserInput();
    initSensor();
    welcomeScreen();
    initServo();
    
    // Initialize PID control variables
    last_error_roll = setpoint_roll;
    last_error_pitch = setpoint_pitch;
    
    rtosInit();
    // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
}

void initSensor(){
    
    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    // Calibrate MPU6050
    //Serial.println("Calibrating MPU6050...");
    //delay(2000);
    //mpu.CalibrateGyro();
    //Serial.println();
    //mpu.CalibrateAccel();
    //Serial.println();
    
    if(!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }

    
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        while (1) delay(10);
    }
    Serial.println(F("BMP280 Found!"));
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
     
}

void rtosInit(){
    // Now set up three tasks to run independently.
    xTaskCreatePinnedToCore(
      TaskGyroSystem
      ,  "TaskGyroSystem"   // A name just for humans
      ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL 
      ,  ARDUINO_RUNNING_CORE);
      
    xTaskCreatePinnedToCore(
      TaskGuidanceSystem
      ,  "TaskGuidanceSystem"   // A name just for humans
      ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL 
      ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
      TaskBasicManagement
      ,  "TaskBasicManagement"   // A name just for humans
      ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL 
      ,  ARDUINO_RUNNING_CORE);
    
    xTaskCreatePinnedToCore(
      TaskCommSystem
      ,  "TaskCommSystem"   // A name just for humans
      ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL 
      ,  ARDUINO_RUNNING_CORE);
}

void loop()
{
  // read the state of the switch/button:
  currentState = digitalRead(SW_PIN);

  if(lastState == HIGH && currentState == LOW) {        // button is pressed
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  } else if(lastState == LOW && currentState == HIGH) { // button is released
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME )
      if (intialState==false){
        Serial.println("A short press is detected");
        display_page++;
        if (display_page>9) display_page=0;
      } 
      else intialState = false;
      
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
      isLongDetected = true;

      if (armed==true) armed = false;
      else armed = true;

    }
  }
  
  // save the the last state
  lastState = currentState;

  delay(100);
}

/*--------------------------------------------------*/
/*------------------- RTOS Tasks -------------------*/
/*--------------------------------------------------*/

void TaskGyroSystem(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    
    for (;;) // A Task shall never return or exit.
    {
        // Calculate Yaw angle from compass data
        getHeading();
        // Calculate pitch angle from acceleration data
        getAngle();
        readBPM280();
        //Calculate PID
        if ((flight_mode == AUTO) || (flight_mode == STABLE)) selfBalancePID();
        else runManual();
        vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskGuidanceSystem(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    
    for (;;) // A Task shall never return or exit.
    {
        //Serial.println("Task 2: Guidance System");
        while (Serial1.available() > 0)
            if (gps.encode(Serial1.read()))
                readGPS();
        if (flight_mode == AUTO){
            if (target_distance>5) DirectionControlPID();
            else gps_roll_angle = 5.0;
            AltitudeControlPID();
            setpoint_roll = gps_roll_angle;
            setpoint_pitch = gps_pitch_angle;
        }
        else{
            setpoint_roll = 0.0;
            setpoint_pitch = 0.0;
        }
        vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskBasicManagement(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    
    for (;;) // A Task shall never return or exit.
    {
        //Serial.println("Task 3: Power Management");
        // Print out debugging information
        showDisplay();
        refreshUserInput();
        //printPID();
        vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskCommSystem(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    
    for (;;) // A Task shall never return or exit.
    {
        //Serial.println("Task 4: Comm System");
        readChannel();
        vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    }
}

/*--------------------------------------------------*/
/*--------------- Startup Function -----------------*/
/*--------------------------------------------------*/

void initGPIO(){
    
    pinMode(CH1_PIN, INPUT);
    attachInterrupt(CH1_PIN, ch1_ISR, CHANGE);
    pinMode(CH2_PIN, INPUT);
    attachInterrupt(CH2_PIN, ch2_ISR, CHANGE);
    pinMode(CH3_PIN, INPUT);
    attachInterrupt(CH3_PIN, ch3_ISR, CHANGE);
    pinMode(CH4_PIN, INPUT);
    attachInterrupt(CH4_PIN, ch4_ISR, CHANGE);
    pinMode(CH5_PIN, INPUT);
    attachInterrupt(CH5_PIN, ch5_ISR, CHANGE);
    pinMode(CH6_PIN, INPUT);
    attachInterrupt(CH6_PIN, ch6_ISR, CHANGE);

    pinMode(SW_PIN, INPUT_PULLUP);
}

void initServo(){
    
    Serial.println("Calibrating Servo...");
    ESP32PWM::allocateTimer(0); //0-3
    ESP32PWM::allocateTimer(1); //0-3
    ESP32PWM::allocateTimer(2); //0-3
    ESP32PWM::allocateTimer(3); //0-3
    
    //AILERON_L_PIN, AILERON_R_PIN,ELEVATOR_PIN, RUDDER_PIN, BLDC_PIN
    //aileronL, aileronR, elevator, rudder, bldc;
    aileronL.setPeriodHertz(50);
    aileronR.setPeriodHertz(50);
    elevator.setPeriodHertz(150);
    rudder.setPeriodHertz(250);
    bldc.setPeriodHertz(330);
    //servo2.setPeriodHertz(50);
    
    // Attach servo to pin 2
    aileronL.attach(AILERON_L_PIN);
    aileronR.attach(AILERON_R_PIN);
    elevator.attach(ELEVATOR_PIN);
    rudder.attach(RUDDER_PIN);
    bldc.attach(BLDC_PIN);
    
    // Set initial position of servo 
    aileronL.write(AILERON_L_ZERO);
    aileronR.write(AILERON_R_ZERO);
    elevator.write(ELEVATOR_ZERO);
    rudder.write(RUDDER_ZERO);
    
    Serial.println("Calibrating ESC...");
    bldc.writeMicroseconds(2000);
    delay(2000);
    bldc.writeMicroseconds(1000);
    delay(2000);
    bldc.writeMicroseconds(1000);
    //delay(2000);
    //bldc.writeMicroseconds(1500);
    
}

void welcomeScreen(){
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(5, 10);
    display.println("AutoPilot");
    display.setTextSize(1);
    display.setCursor(25, 35);
    display.println("Version 1.0");
    display.display();
    
}

void displaySensorDetails(void)
{
    sensor_t sensor;
    mag.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/*--------------------------------------------------*/
/*----------------  PID Function   -----------------*/
/*--------------------------------------------------*/

void selfBalancePID(){
  
    // Calculate errors for PID control
    error_roll = setpoint_roll + roll;
    error_pitch = setpoint_pitch + pitch;
    error_yaw = setpoint_yaw + yaw;
    
    if ((error_roll>5.0)||(error_roll<-5.0)||(error_pitch>5.0)||(error_pitch<-5.0)) {
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
    }
    else{
        pixel.setPixelColor(0, pixel.Color(0, 255, 0));
        pixel.show();
    }
  
    // Calculate integral and derivative components of PID control
    integral_roll += error_roll;
    integral_pitch += error_pitch;
    integral_yaw += error_yaw;
    
    derivative_roll = error_roll - last_error_roll;
    derivative_pitch = error_pitch - last_error_pitch;
    derivative_yaw = error_yaw - last_error_yaw;
    
    last_error_roll = error_roll;
    last_error_pitch = error_pitch;
    last_error_yaw = error_yaw;

    if (integral_roll > 10000.0) integral_roll = 10000.0;
    else if (integral_roll < -10000.0) integral_roll = -10000.0;

    if (integral_pitch > 10000.0) integral_pitch = 10000.0;
    else if (integral_pitch < -10000.0) integral_pitch = -10000.0;

    if (integral_yaw > 10000.0) integral_yaw = 10000.0;
    else if (integral_yaw < -10000.0) integral_yaw = -10000.0;
    
    // Calculate output of PID control
    output_roll   = rollKp * error_roll + rollKi * integral_roll + rollKd * derivative_roll;
    output_pitch  = pitchKp * error_pitch + pitchKi * integral_pitch + pitchKd * derivative_pitch;
    output_yaw    = yawKp * error_yaw + yawKi * integral_yaw + yawKd * derivative_yaw;
  
    if (output_roll > 1000) output_roll = 1000;
    else if (output_roll < -1000) output_roll = -1000;
  
    if (output_pitch > 1000) output_pitch = 1000;
    else if (output_pitch < -1000) output_pitch = -1000;

    if (output_yaw > 1000) output_yaw = 1000;
    else if (output_yaw < -1000) output_yaw = -1000;
    
    int rx_roll   = 0;
    int rx_pitch  = 0;
    int rx_yaw    = 0;
    
    if (ch1 >= MIN_SIGNAL_WIDTH && ch1 <= MAX_SIGNAL_WIDTH) rx_roll = map(ch1, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -1000, 1000); 
    if (ch2 >= MIN_SIGNAL_WIDTH && ch2 <= MAX_SIGNAL_WIDTH) rx_pitch = map(ch2, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -1000, 1000);
    if (ch4 >= MIN_SIGNAL_WIDTH && ch4 <= MAX_SIGNAL_WIDTH) rx_yaw = map(ch4, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -1000, 1000);

    int aileron_angle = 0;
    int elevator_angle = 0;
    int rudder_angle = 0;
    
    if (flight_mode == STABLE){
        aileron_angle   = map(output_roll+rx_roll, -1000, 1000, -AILERON_DISPLACEMENT_PID, AILERON_DISPLACEMENT_PID); //0+SERVO_DISPLACEMENT_PID, 180-SERVO_OFFSET_PID
        elevator_angle  = map(output_pitch+rx_pitch, -1000, 1000, -ELEVATOR_DISPLACEMENT_PID, ELEVATOR_DISPLACEMENT_PID); //180-SERVO_OFFSET_PID, 0+SERVO_OFFSET_PID
        rudder_angle    = map(output_yaw+rx_yaw, -1000, 1000, -RUDDER_DISPLACEMENT_PID, RUDDER_DISPLACEMENT_PID); //180-SERVO_OFFSET_PID, 0+SERVO_OFFSET_PID
        
        //aileronL, aileronR, elevator, rudder, bldc
        //AILERON_L_ZERO AILERON_R_ZERO ELEVATOR_ZERO RUDDER_ZERO
        // Move servo to new angle
        aileronL.write(AILERON_L_ZERO + aileron_angle);
        aileronR.write(AILERON_R_ZERO + aileron_angle);
        elevator.write(ELEVATOR_ZERO + elevator_angle);
        rudder.write(RUDDER_ZERO - rudder_angle);
        
        if (armed) bldc.writeMicroseconds(ch3);
        else bldc.writeMicroseconds(1000);
    }
    else if (flight_mode == AUTO){
        aileron_angle   = map(output_roll, -1000, 1000, -AILERON_DISPLACEMENT_PID, AILERON_DISPLACEMENT_PID); //0+SERVO_OFFSET_PID, 180-SERVO_OFFSET_PID
        elevator_angle  = map(output_pitch, -1000, 1000, -ELEVATOR_DISPLACEMENT_PID, ELEVATOR_DISPLACEMENT_PID); //180-SERVO_OFFSET_PID, 0+SERVO_OFFSET_PID
        rudder_angle    = map(output_yaw, -1000, 1000, -RUDDER_DISPLACEMENT_PID, RUDDER_DISPLACEMENT_PID); //180-SERVO_OFFSET_PID, 0+SERVO_OFFSET_PID
        
        //aileronL, aileronR, elevator, rudder, bldc
        //AILERON_L_ZERO AILERON_R_ZERO ELEVATOR_ZERO RUDDER_ZERO
        // Move servo to new angle
        aileronL.write(AILERON_L_ZERO + aileron_angle);
        aileronR.write(AILERON_R_ZERO + aileron_angle);
        elevator.write(ELEVATOR_ZERO + elevator_angle);
        rudder.write(RUDDER_ZERO - rudder_angle);
        //bldc.writeMicroseconds(ch3);
    }
    
    
    if (DEBUG_BPID){
        Serial.print("Pitch: "); 
        Serial.print(pitch);
        Serial.print(" | Roll: ");
        Serial.print(roll);
        //Serial.print(" | Yaw: ");
        //Serial.print(crnt_bearing);
        
        //Serial.print(" | ErrorRoll: ");
        //Serial.print(error_roll);
        //Serial.print(" | ErrorPitch: ");
        //Serial.print(error_pitch);
        //Serial.print(" | IntegralRoll: ");
        //Serial.print(integral_roll);
        //Serial.print(" | IntegralPitch: ");
        //Serial.print(integral_pitch);
        //Serial.print(" | DerivativeRoll: ");
        //Serial.print(derivative_roll);
        //Serial.print(" | DerivativePitch: ");
        //Serial.print(derivative_pitch);
        
        Serial.print(" | OutputRoll: ");
        Serial.print(output_roll);
        Serial.print(" | OutputPitch: ");
        Serial.print(output_pitch);
        Serial.print(" | RX_Roll: ");
        Serial.print(rx_roll);
        Serial.print(" | RX_Pitch: ");
        Serial.print(rx_pitch);
        Serial.print(" | Aileron: ");
        Serial.print(aileron_angle);
        Serial.print(" | Elevator: ");
        Serial.print(elevator_angle);
        Serial.println();
    }
}

void DirectionControlPID(){
  
    float heading = target_heading-mag_heading;
    // Calculate errors for PID control
    error_heading = setpoint_heading - heading;

    // Calculate integral and derivative components of PID control
    integral_heading += error_heading;
    derivative_heading = error_heading - last_error_heading;
    last_error_heading = error_heading;

    if (integral_heading > 10000.0) integral_heading = 10000.0;
    else if (integral_heading < -10000.0) integral_heading = -10000.0;

    // Calculate output of PID control
    output_heading = bankKp * error_heading + bankKi * integral_heading + bankKd * derivative_heading;

    if (output_heading > 1000) output_heading = 1000;
    else if (output_heading < -1000) output_heading = -1000;

    // Map output to angle
    int gps_roll   = map(output_heading, -1000, 1000, -150, 150);

    // Limit angle to within acceptable range
    gps_roll  = constrain(gps_roll, -150, 150);
    gps_roll_angle = -(float) gps_roll/10.0;

    //int rudder_angle = 0;
    //rudder_angle  = map(output_heading, -1000, 1000, -90, 90); //180-SERVO_OFFSET_PID, 0+SERVO_OFFSET_PID
    //rudder.write(RUDDER_ZERO + rudder_angle);

    if (DEBUG_NPID){
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(" | Error: ");
        Serial.print(error_heading);
        Serial.print(" | Integral: ");
        Serial.print(integral_heading);
        Serial.print(" | Derivative: ");
        Serial.print(derivative_heading);
        Serial.print(" | Output: ");
        Serial.print(output_heading);
        Serial.print(" | Roll: ");
        Serial.print(gps_roll_angle);
        Serial.println();
    }
}

//altitude
void AltitudeControlPID(){
  
    float altDiff = target_alt-current_alt;
    // Calculate errors for PID control
    error_altitude = setpoint_altitude - altDiff;

    // Calculate integral and derivative components of PID control
    integral_altitude += error_altitude;
    derivative_altitude = error_altitude - last_error_altitude;
    last_error_altitude = error_altitude;

    if (integral_altitude > 10000.0) integral_altitude = 10000.0;
    else if (integral_altitude < -10000.0) integral_altitude = -10000.0;

    // Calculate output of PID control
    output_altitude = altitudeKp * error_altitude + altitudeKi * integral_altitude + altitudeKd * derivative_altitude;

    if (output_altitude > 1000) output_altitude = 1000;
    else if (output_altitude < -1000) output_altitude = -1000;

    // Map output to angle
    int gps_pitch   = map(output_altitude, -1000, 1000, -150, 150);

    // Limit angle to within acceptable range
    gps_pitch  = constrain(gps_pitch, -150, 150);
    gps_pitch_angle = -(float) gps_pitch/10.0;

    if (DEBUG_NPID){
        Serial.print("Altitude: ");
        Serial.print(altDiff);
        Serial.print(" | Error: ");
        Serial.print(error_altitude);
        Serial.print(" | Integral: ");
        Serial.print(integral_altitude);
        Serial.print(" | Derivative: ");
        Serial.print(derivative_altitude);
        Serial.print(" | Output: ");
        Serial.print(output_altitude);
        Serial.print(" | Pitch: ");
        Serial.print(gps_pitch_angle);
        Serial.println();
    }
}


/*--------------------------------------------------*/
/*---------------- Module Function -----------------*/
/*--------------------------------------------------*/

void showDisplay(){
  
  if (display_page==0){
    display.clearDisplay(); //Roll, Pitch, and Yaw
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    display.setCursor(5, 7);
    display.print("Pitch");
    display.setCursor(47, 7);
    display.print("Yaw");
    display.setCursor(89, 7);
    display.print("Roll");

    display.setCursor(5, 17);
    display.print(String(pitch,2));
    display.setCursor(47, 17);
    display.print(String(mag_heading,2));
    display.setCursor(89, 17);
    display.print(String(roll,2));

    display.setCursor(5, 27);
    display.print(String(setpoint_pitch,2));
    display.setCursor(47, 27);
    display.print(String(target_heading,2));
    display.setCursor(89, 27);
    display.print(String(setpoint_roll,2));
    
    String line3 = "GPS:" + String(current_lat,4) + "," + String(current_lon,4);
    display.setCursor(5, 37);
    display.print(line3);

    String line4 = "TB:" + String(target_heading-mag_heading,2) + " TD:" + String(target_distance,1) + "m";
    display.setCursor(5, 47);
    display.println(line4);
    
    //String line5 = "RX: " + String(ch1) + " " + String(ch2) + " " + String(ch3) + " " + String(ch4) + " " + String(ch5) + " " + String(ch6);
    //display.setCursor(5, 47);
    //display.println(line5); gps_roll_angle
    
    display.display();
  }
  else if (display_page==1){
    
    display.clearDisplay(); //Roll, Pitch, and Yaw
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "Hdop: " + String(gps_hdop,3);
    String line2 = "LAT: " + String(current_lat,10);
    String line3 = "LON: " + String(current_lon,10);
    String line4 = "ALT: " + String(current_alt,3);
    String line5 = "ARMED: " + String(armed);
    
    display.setCursor(5, 7);
    display.print(line1);
    display.setCursor(5, 17);
    display.print(line2);
    display.setCursor(5, 27);
    display.print(line3);
    display.setCursor(5, 37);
    display.println(line4);
    display.setCursor(5, 47);
    display.println(line5);
    display.display();
    
  }
  else if (display_page==2){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = String(target_heading,1) + " N, " + String(target_distance,1) + " m";
    String line2 = "LAT: " + String(target_lat,10);
    String line3 = "LON: " + String(target_lon,10);
    String line4 = "ALT: " + String(target_alt,3);
    
    display.setCursor(5, 7);
    display.print("Target Position");
    display.setCursor(5, 17);
    display.print(line1);
    display.setCursor(5, 27);
    display.print(line2);
    display.setCursor(5, 37);
    display.println(line3);
    display.setCursor(5, 47);
    display.println(line4);
    display.display();
    
  }
  else if (display_page==3){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "Heading: " + String(mag_heading,2);
    String line2 = "Altitude: " + String(bpm_altitude,2);
    String line3 = "Pressure: " + String(bpm_pressure,2);
    String line4 = "Temp.: " + String(bpm_temperature,2);
    
    display.setCursor(5, 7);
    display.print(line1);
    display.setCursor(5, 17);
    display.print(line2);
    display.setCursor(5, 27);
    display.print(line3);
    display.setCursor(5, 37);
    display.println(line4);
    display.display();
    
  }
  else if (display_page==4){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "Kp: " + String(rollKp,3);
    String line2 = "Ki: " + String(rollKi,3);
    String line3 = "Kd: " + String(rollKd,3);
    
    display.setCursor(5, 7);
    display.print("PID ROLL");
    display.setCursor(5, 20);
    display.print(line1);
    display.setCursor(5, 30);
    display.print(line2);
    display.setCursor(5, 40);
    display.print(line3);
    display.display();
    
  }
  else if (display_page==5){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "Kp: " + String(pitchKp,3);
    String line2 = "Ki: " + String(pitchKi,3);
    String line3 = "Kd: " + String(pitchKd,3);
    //String line4 = "Temp.: " + String(bpm_temperature,2);

    display.setCursor(5, 7);
    display.print("PID PITCH");
    display.setCursor(5, 20);
    display.print(line1);
    display.setCursor(5, 30);
    display.print(line2);
    display.setCursor(5, 40);
    display.print(line3);
    display.display();
    
  }
  else if (display_page==6){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "Kp: " + String(yawKp,3);
    String line2 = "Ki: " + String(yawKi,3);
    String line3 = "Kd: " + String(yawKd,3);
    
    display.setCursor(5, 7);
    display.print("PID YAW");
    display.setCursor(5, 20);
    display.print(line1);
    display.setCursor(5, 30);
    display.print(line2);
    display.setCursor(5, 40);
    display.print(line3);
    display.display();
    
  }
  else if (display_page==9){
    
    display.clearDisplay();
    display.drawRect(1, 1, display.width()-2*1, display.height()-2*1, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    String line1 = "CH1: " + String(ch1) + " CH6: " + String(ch6);
    String line2 = "CH2: " + String(ch2) + " CH7: " + String(0);
    String line3 = "CH3: " + String(ch3) + " CH8: " + String(0);
    String line4 = "CH4: " + String(ch4) + " CH9: " + String(0);
    String line5 = "CH5: " + String(ch5) + " CH10: " + String(0);

    display.setCursor(5, 7);
    display.print(line1);
    display.setCursor(5, 17);
    display.print(line2);
    display.setCursor(5, 27);
    display.print(line3);
    display.setCursor(5, 37);
    display.print(line4);
    display.setCursor(5, 47);
    display.println(line5);
    display.display();
    
  }
  else{
    display.clearDisplay();
    display.display();
  }
}

void readChannel(){
    
    //flight_mode = AUTO/STABLE/REMOTE
    if ((ch6>=800)&&(ch6<1300))flight_mode = STABLE;
    else if ((ch6>=1300)&&(ch6<1800)) flight_mode = REMOTE;
    else if ((ch6>=1800)&&(ch6<2200)) flight_mode = AUTO;
    else flight_mode = STABLE;
    
    if (DEBUG_RF){
        Serial.print("Channel: "); 
        Serial.print(ch1); 
        Serial.print(" | "); 
        Serial.print(ch2); 
        Serial.print(" | "); 
        Serial.print(ch3); 
        Serial.print(" | "); 
        Serial.print(ch4); 
        Serial.print(" | "); 
        Serial.print(ch5); 
        Serial.print(" | "); 
        Serial.print(ch6); 
        Serial.print(" | "); 
        Serial.println();
    }
    
}

void resetPID(){
  
    setpoint_roll = 0.0;    // target angle  
    setpoint_pitch = 0.0;
    error_roll = 0.0;       // difference between target and actual angle    
    error_pitch = 0.0;
    last_error_roll = 0.0;  // previous error value
    last_error_pitch = 0.0;
    integral_roll = 0.0;    // sum of errors over time
    integral_pitch = 0.0;
    derivative_roll = 0.0;  // change in error over time
    derivative_pitch = 0.0;
    output_roll = 0.0;      // control variable for PID controller
    output_pitch = 0.0;

    setpoint_heading = 0.0;    // target angle  
    error_heading = 0.0;       // difference between target and actual angle   
    last_error_heading = 0.0;  // previous error value
    integral_heading = 0.0;    // sum of errors over time
    derivative_heading = 0.0;  // change in error over time
    output_heading = 0.0;      // control variable for PID controller

    setpoint_altitude = 0.0;    // target altitude  
    error_altitude = 0.0;       // difference between target and actual altitude   
    last_error_altitude = 0.0;  // previous error value
    integral_altitude = 0.0;    // sum of errors over time
    derivative_altitude = 0.0;  // change in error over time
    output_altitude = 0.0;      // control variable for PID controller
    
}

void runManual(){

    pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    pixel.show();

    resetPID();
    
    // Map the FlySky input values to the range of -100 to 100
    int throttle  = map(ch3, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, 1000, 2000);
    int roll      = map(ch1, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -AILERON_DISPLACEMENT_ML, AILERON_DISPLACEMENT_ML);  //1:roll, 2:pitch, 4:yaw
    int pitch     = map(ch2, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -ELEVATOR_DISPLACEMENT_ML, ELEVATOR_DISPLACEMENT_ML);
    int yaw       = map(ch4, MIN_SIGNAL_WIDTH, MAX_SIGNAL_WIDTH, -RUDDER_DISPLACEMENT_ML, RUDDER_DISPLACEMENT_ML);
    
    //aileronL, aileronR, elevator, rudder, bldc
    //AILERON_L_ZERO AILERON_R_ZERO ELEVATOR_ZERO RUDDER_ZERO
    // Move servo to new angle
    aileronL.write(AILERON_L_ZERO + roll);
    aileronR.write(AILERON_R_ZERO + roll);
    elevator.write(ELEVATOR_ZERO + pitch);
    rudder.write(RUDDER_ZERO - yaw);
    
    if (armed) bldc.writeMicroseconds(ch3);
    else bldc.writeMicroseconds(1000);

    if (DEBUG_MANUAL){
        Serial.print("Manual: Aileron: ");
        Serial.print(AILERON_L_ZERO + roll);
        Serial.print(" | Elevator: ");
        Serial.print(ELEVATOR_ZERO + pitch);
        Serial.print(" | Rudder: ");
        Serial.print(RUDDER_ZERO - yaw);
        Serial.print(" | Throttle: ");
        Serial.print(throttle);
        Serial.println();
    }
    // Move servo to new angle
    //servo1.write(180-elevon_right);
    //servo2.write(elevon_left);
}

void readGPS(){
    
    if (gps.location.isValid() && gps.altitude.isValid() && gps.hdop.isValid()){
    
        //gps.hdop.hdop(), gps.hdop.isValid()
        
        
        gps_hdop = gps.hdop.hdop();
        if (gps_hdop<2.0){
            current_lat = gps.location.lat();
            current_lon = gps.location.lng();
            current_alt = gps.altitude.meters();
            // Calculate ground speed and airspeed
            ground_speed = gps.speed.kmph(); // in km/h
            // Calculate bearing and distance to target
            target_distance = TinyGPSPlus::distanceBetween(current_lat, current_lon, target_lat, target_lon); // in m
            target_heading = TinyGPSPlus::courseTo(current_lat, current_lon, target_lat, target_lon); // in degrees
        }
        
        if (DEBUG_GPS){
            Serial.print("GPS- hdop:");
            Serial.print(gps_hdop, 2);
            Serial.print(F(", Lat:"));
            Serial.print(current_lat, 6);
            Serial.print(F(", Lon:"));
            Serial.print(current_lon, 6);
            Serial.print(F(", Alt:"));
            Serial.print(current_alt, 2);
            Serial.print(F(", D:"));
            Serial.print(target_distance, 2);
            Serial.print(F(", B:"));
            Serial.print(target_heading, 2);
            Serial.print(F(", S:"));
            Serial.print(ground_speed, 2);
            Serial.println();
        }
        gps_ready = true;
        //delay(500);
    }
    else{
      gps_ready = false;
    }
}

void readBPM280(){
    
    bpm_altitude = bmp.readAltitude(1013.25);
    bpm_pressure = bmp.readPressure();
    bpm_temperature = bmp.readTemperature();

    if (DEBUG_BPM){
        Serial.print(F("Temperature = "));
        Serial.print(bpm_temperature);
        Serial.println(" ºC");
    
        Serial.print(F("Pressure = "));
        Serial.print(bpm_pressure);
        Serial.println(" Pa");
    
        Serial.print(F("Approx altitude = "));
        Serial.print(bpm_altitude); /* Adjusted to local forecast! */
        Serial.println(" m");
    }
}

void calibrateAccel() {

  int ax, ay, az;
  int num_samples = 100;
  float ax_offset = 0, ay_offset = 0, az_offset = 0;

  // Read and discard first set of samples
  for (int i = 0; i < num_samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      delay(10);
  }

  // Read and average a set of samples
  for (int i = 0; i < num_samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      ax_offset += ax;
      ay_offset += ay;
      az_offset += az;
      delay(10);
  }
  ax_offset /= num_samples;
  ay_offset /= num_samples;
  az_offset /= num_samples;

  // Update calibration offsets
  accel_offset_x = ax_offset;
  accel_offset_y = ay_offset;
  accel_offset_z = az_offset;

  Serial.print("Accelerometer calibration complete: ");
  Serial.print("X offset = ");
  Serial.print(accel_offset_x);
  Serial.print(", Y offset = ");
  Serial.print(accel_offset_y);
  Serial.print(", Z offset = ");
  Serial.println(accel_offset_z);
}

void calibrateGyro() {
  
  int gx, gy, gz;
  int num_samples = 100;
  float gx_offset = 0, gy_offset = 0, gz_offset = 0;

  // Read and discard first set of samples
  for (int i = 0; i < num_samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;
      delay(10);
  }

  // Read and average a set of samples
  for (int i = 0; i < num_samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;
      gx_offset += gx;
      gy_offset += gy;
      gz_offset += gz;
      delay(10);
  }
  gx_offset /= num_samples;
  gy_offset /= num_samples;
  gz_offset /= num_samples;

  // Update calibration offsets
  gyro_offset_x = gx_offset;
  gyro_offset_y = gy_offset;
  gyro_offset_z = gz_offset;

  Serial.print("Gyro calibration complete: ");
  Serial.print("X offset = ");
  Serial.print(gyro_offset_x);
  Serial.print(", Y offset = ");
  Serial.print(gyro_offset_y);
  Serial.print(", Z offset = ");
  Serial.println(gyro_offset_z);
}
/*
void getAngle() {
    
    // Read sensor inputs from MPU6050
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accel_x = a.acceleration.x;
    accel_y = a.acceleration.y;
    accel_z = a.acceleration.z;

    gyro_x = g.gyro.x;
    gyro_y = g.gyro.y;
    gyro_z = g.gyro.z;
    
    // Apply calibration offsets
    accel_x -= accel_offset_x;
    accel_y -= accel_offset_y;
    accel_z -= accel_offset_z;
    gyro_x -= gyro_offset_x;
    gyro_y -= gyro_offset_y;
    gyro_z -= gyro_offset_z;
    
    // Convert gyro readings to radians per second
    float gyro_x_rad_per_sec = (gyro_x / 131.0) * DEG_TO_RAD;
    float gyro_y_rad_per_sec = (gyro_y / 131.0) * DEG_TO_RAD;
    float gyro_z_rad_per_sec = (gyro_z / 131.0) * DEG_TO_RAD;

    float accel_x_g = (float)(accel_x / 16384.0);
    float accel_y_g = (float)(accel_y / 16384.0);
    float accel_z_g = (float)(accel_z / 16384.0);
    
    // Calculate elapsed time since last sensor reading
    uint32_t current_time = millis(); //micros(); millis(); 000
    float time_elapsed = (current_time - previous_time) / 1000.0; 
    
    // Calculate complementary filter coefficients
    float alpha = 0.94;
    float beta = 1 - alpha;
    
    // Calculate pitch and roll angles from accelerometer readings
    float roll_acc = atan2(accel_y, accel_z) * RAD_TO_DEG;
    //float roll_acc = atan2(-accel_y_g, sqrt(accel_x_g*accel_x_g + accel_z_g*accel_z_g)) * RAD_TO_DEG;
    float pitch_acc = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;
    //float pitch_acc = atan2(-accel_x_g, sqrt(accel_y_g*accel_y_g + accel_z_g*accel_z_g)) * RAD_TO_DEG;
    
    // Apply complementary filter to combine accelerometer and gyro readings
    pitch = alpha * (pitch + gyro_y_rad_per_sec * time_elapsed) + beta * pitch_acc;
    roll = alpha * (roll + gyro_x_rad_per_sec * time_elapsed) + beta * roll_acc;
    yaw = yaw + gyro_z_rad_per_sec * time_elapsed;
    Serial.println(yaw);
    
    // Keep yaw angle between -180 and 180 degrees
    if (yaw > 180) {
        yaw -= 360;
    } else if (yaw < -180) {
        yaw += 360;
    }

    if (DEBUG_SENSOR) {
        // Print the current orientation angles
        Serial.print("Pitch: ");
        Serial.print(pitch);
        Serial.print(" Roll: ");
        Serial.print(roll);
        Serial.print(" Yaw: ");
        Serial.println(yaw);
    }
    // Save current time for next iteration
    previous_time = current_time;
}
*/

void getAngle() {
  
    timePrev = currentTime;  // the previous time is stored before the actual time read
    currentTime = millis();  // actual time read
    elapsedTime = (currentTime - timePrev) / 1000;
    
    // put your main code here, to run repeatedly:
    // Read sensor inputs from MPU6050
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accel_x = a.acceleration.x;
    accel_y = a.acceleration.y;
    accel_z = a.acceleration.z;

    gyro_x = g.gyro.x;
    gyro_y = g.gyro.y;
    gyro_z = g.gyro.z;
    
    // Apply calibration offsets
    accel_x -= accel_offset_x;
    accel_y -= accel_offset_y;
    accel_z -= accel_offset_z;
    gyro_x -= gyro_offset_x;
    gyro_y -= gyro_offset_y;
    gyro_z -= gyro_offset_z;
    
    // Apply calibration offsets
    float gyro_x_rad = (gyro_x / gyro_sensitivity) * M_PI / 180.0;
    float gyro_y_rad = (gyro_y / gyro_sensitivity) * M_PI / 180.0;
    float gyro_z_rad = (gyro_z / gyro_sensitivity) * M_PI / 180.0;
    float accel_x_g = (float)accel_x / accl_sensitivity;
    float accel_y_g = (float)accel_y / accl_sensitivity;
    float accel_z_g = (float)accel_z / accl_sensitivity;
  
    // Calculate the pitch and roll angles using sensor fusion
    
    float acc_pitch = 180 * atan2(accel_x_g, sqrt(accel_y_g*accel_y_g + accel_z_g*accel_z_g)) / M_PI;
    float acc_roll = 180 * atan2(accel_y_g, accel_z_g) / M_PI;
    float gyro_pitch = acc_pitch + gyro_x_rad * elapsedTime;
    float gyro_roll = acc_roll + gyro_y_rad * elapsedTime;
    float gyro_yaw = gyro_z_rad * elapsedTime;
    
    // Integrate the yaw rate to obtain the yaw angle
    yaw += gyro_yaw;
    pitch = gyro_pitch;
    roll =gyro_roll;

    float yawRate = (float)gyro_z / 131.0; // Convert raw gyroscope data to degrees per second
    yaw = yawRate * 1000;
  
    //Serial.print("Xº: ");
    //Serial.print(roll);
    //Serial.print("   |   ");
    //Serial.print("Yº: ");
    //Serial.print(pitch);
    //Serial.println(" ");
}


void getHeading(){
    //crnt_bearing = 0.0;
    if(mag.begin())
    {
        /* Get a new sensor event */ 
        sensors_event_t event; 
        mag.getEvent(&event);   
        
        float heading = atan2(event.magnetic.y, event.magnetic.x);
        //float declinationAngle = 0.019; //0.019
        //heading += declinationAngle;
  
        //heading -= PI; //if chip is on bottom layer
        
        // Correct for when signs are reversed.
        if(heading < 0)
          heading += 2*PI;
          
        // Check for wrap due to addition of declination.
        if(heading > 2*PI)
          heading -= 2*PI;
          
        // Convert radians to degrees for readability.
        mag_heading = heading * 180/M_PI; 
        
    }
    if (DEBUG_SENSOR) {
      Serial.print("MAG Heading (degrees): "); 
      Serial.println(mag_heading);
    }
}

double calculate_desired_roll(double target_distance, double target_heading) {
    // Convert target heading to radians
    double target_heading_rad = target_heading * PI / 180.0;
    
    // Calculate the desired roll angle based on the target distance and heading
    double desired_roll = atan(target_distance / (2.0 * 6371000.0)) * 180.0 / PI * cos(target_heading_rad);
    
    // Return the desired roll angle
    return desired_roll;
}

double calculate_desired_pitch(double target_distance, double target_heading, double airspeed, double altitude) {
    // Convert target heading to radians
    double target_heading_rad = target_heading * PI / 180.0;
    
    // Calculate the desired pitch angle based on the target distance, heading, airspeed, and altitude
    double desired_pitch = atan((altitude * tan(target_heading_rad)) / (target_distance + (airspeed * 0.2))) * 180.0 / PI;
    
    // Return the desired pitch angle
    return desired_pitch;
}

/*
// Calculate desired pitch, yaw, and roll angles based on target distance and heading
void calculate_desired_pitch_yaw_roll(double target_distance, double target_heading, double airspeed) {
    // Calculate desired yaw angle based on target heading
    desired_yaw = target_heading;
    // Calculate desired pitch angle based on target distance
    desired_pitch = atan2(target_distance, 2 * (altitude + 5 * 0.2) * 57.2958);
    // Calculate desired roll angle based on desired yaw and pitch angles
    double heading_error = wrap_180(desired_yaw - yaw) / 57.2958;
    desired_roll = asin(heading_error * 9.81 * 0.2 / airspeed) * 57.2958;
}
*/


void startWebServer(){
  
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputLatitude=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputLatitude value on <ESP_IP>/get?inputLatitude=<inputMessage>
    if (request->hasParam(PARAM_LAT)) {
      inputMessage = request->getParam(PARAM_LAT)->value();
      writeFile(SPIFFS, "/inputLatitude.txt", inputMessage.c_str());
    }
    // GET inputLongitude value on <ESP_IP>/get?inputLongitude=<inputMessage>
    else if (request->hasParam(PARAM_LON)) {
      inputMessage = request->getParam(PARAM_LON)->value();
      writeFile(SPIFFS, "/inputLongitude.txt", inputMessage.c_str());
    }
    // GET inputAltitude value on <ESP_IP>/get?inputAltitude=<inputMessage>
    else if (request->hasParam(PARAM_ALT)) {
      inputMessage = request->getParam(PARAM_ALT)->value();
      writeFile(SPIFFS, "/inputAltitude.txt", inputMessage.c_str());
    }
    // GET inputRollKp value on <ESP_IP>/get?inputRollKp=<inputMessage>
    else if (request->hasParam(PARAM_RKP)) {
      inputMessage = request->getParam(PARAM_RKP)->value();
      writeFile(SPIFFS, "/inputRollKp.txt", inputMessage.c_str());
    }
    // GET inputRollKi value on <ESP_IP>/get?inputRollKi=<inputMessage>
    else if (request->hasParam(PARAM_RKI)) {
      inputMessage = request->getParam(PARAM_RKI)->value();
      writeFile(SPIFFS, "/inputRollKi.txt", inputMessage.c_str());
    }
    // GET inputRollKd value on <ESP_IP>/get?inputRollKd=<inputMessage>
    else if (request->hasParam(PARAM_RKD)) {
      inputMessage = request->getParam(PARAM_RKD)->value();
      writeFile(SPIFFS, "/inputRollKd.txt", inputMessage.c_str());
    }
    // GET inputYawKp value on <ESP_IP>/get?inputYawKp=<inputMessage>
    else if (request->hasParam(PARAM_YKP)) {
      inputMessage = request->getParam(PARAM_YKP)->value();
      writeFile(SPIFFS, "/inputYawKp.txt", inputMessage.c_str());
    }
    // GET inputYawKi value on <ESP_IP>/get?inputYawKi=<inputMessage>
    else if (request->hasParam(PARAM_YKI)) {
      inputMessage = request->getParam(PARAM_YKI)->value();
      writeFile(SPIFFS, "/inputYawKi.txt", inputMessage.c_str());
    }
    // GET inputYawKd value on <ESP_IP>/get?inputYawKd=<inputMessage>
    else if (request->hasParam(PARAM_YKD)) {
      inputMessage = request->getParam(PARAM_YKD)->value();
      writeFile(SPIFFS, "/inputYawKd.txt", inputMessage.c_str());
    }
    // GET inputPitchKp value on <ESP_IP>/get?inputPitchKp=<inputMessage>
    else if (request->hasParam(PARAM_PKP)) {
      inputMessage = request->getParam(PARAM_PKP)->value();
      writeFile(SPIFFS, "/inputPitchKp.txt", inputMessage.c_str());
    }
    // GET inputPitchKi value on <ESP_IP>/get?inputPitchKi=<inputMessage>
    else if (request->hasParam(PARAM_PKI)) {
      inputMessage = request->getParam(PARAM_PKI)->value();
      writeFile(SPIFFS, "/inputPitchKi.txt", inputMessage.c_str());
    }
    // GET inputPitchKd value on <ESP_IP>/get?inputPitchKd=<inputMessage>
    else if (request->hasParam(PARAM_PKD)) {
      inputMessage = request->getParam(PARAM_PKD)->value();
      writeFile(SPIFFS, "/inputPitchKd.txt", inputMessage.c_str());
    }
    // GET inputBankKp value on <ESP_IP>/get?inputBankKp=<inputMessage>
    else if (request->hasParam(PARAM_BKP)) {
      inputMessage = request->getParam(PARAM_BKP)->value();
      writeFile(SPIFFS, "/inputBankKp.txt", inputMessage.c_str());
    }
    // GET inputBankKi value on <ESP_IP>/get?inputBankKi=<inputMessage>
    else if (request->hasParam(PARAM_BKI)) {
      inputMessage = request->getParam(PARAM_BKI)->value();
      writeFile(SPIFFS, "/inputBankKi.txt", inputMessage.c_str());
    }
    // GET inputBankKd value on <ESP_IP>/get?inputBankKd=<inputMessage>
    else if (request->hasParam(PARAM_BKD)) {
      inputMessage = request->getParam(PARAM_BKD)->value();
      writeFile(SPIFFS, "/inputBankKd.txt", inputMessage.c_str());
    }
    // GET inputAltitudeKp value on <ESP_IP>/get?inputAltitudeKp=<inputMessage>
    else if (request->hasParam(PARAM_AKP)) {
      inputMessage = request->getParam(PARAM_AKP)->value();
      writeFile(SPIFFS, "/inputAltitudeKp.txt", inputMessage.c_str());
    }
    // GET inputAltitudeKi value on <ESP_IP>/get?inputAltitudeKi=<inputMessage>
    else if (request->hasParam(PARAM_AKI)) {
      inputMessage = request->getParam(PARAM_AKI)->value();
      writeFile(SPIFFS, "/inputAltitudeKi.txt", inputMessage.c_str());
    }
    // GET inputAltitudeKd value on <ESP_IP>/get?inputAltitudeKd=<inputMessage>
    else if (request->hasParam(PARAM_AKD)) {
      inputMessage = request->getParam(PARAM_AKD)->value();
      writeFile(SPIFFS, "/inputAltitudeKd.txt", inputMessage.c_str());
    }
    // GET inputAileroLzero value on <ESP_IP>/get?inputAileroLzero=<inputMessage>
    else if (request->hasParam(PARAM_ALZ)) {
      inputMessage = request->getParam(PARAM_ALZ)->value();
      writeFile(SPIFFS, "/inputAileroLzero.txt", inputMessage.c_str());
    }
    // GET inputAileroRzero value on <ESP_IP>/get?inputAileroRzero=<inputMessage>
    else if (request->hasParam(PARAM_ARZ)) {
      inputMessage = request->getParam(PARAM_ARZ)->value();
      writeFile(SPIFFS, "/inputAileroRzero.txt", inputMessage.c_str());
    }
    // GET inputElevatorZero value on <ESP_IP>/get?inputElevatorZero=<inputMessage>
    else if (request->hasParam(PARAM_ELZ)) {
      inputMessage = request->getParam(PARAM_ELZ)->value();
      writeFile(SPIFFS, "/inputElevatorZero.txt", inputMessage.c_str());
    }
    // GET inputRudderZero value on <ESP_IP>/get?inputRudderZero=<inputMessage>
    else if (request->hasParam(PARAM_RUZ)) {
      inputMessage = request->getParam(PARAM_RUZ)->value();
      writeFile(SPIFFS, "/inputRudderZero.txt", inputMessage.c_str());
    }
    // GET inputAileroLMaxAngleMNL value on <ESP_IP>/get?inputAileroLMaxAngleMNL=<inputMessage>
    else if (request->hasParam(PARAM_ALMM)) {
      inputMessage = request->getParam(PARAM_ALMM)->value();
      writeFile(SPIFFS, "/inputAileroLMaxAngleMNL.txt", inputMessage.c_str());
    }
    // GET inputAileroRMaxAngleMNL value on <ESP_IP>/get?inputAileroRMaxAngleMNL=<inputMessage>
    else if (request->hasParam(PARAM_ARMM)) {
      inputMessage = request->getParam(PARAM_ARMM)->value();
      writeFile(SPIFFS, "/inputAileroRMaxAngleMNL.txt", inputMessage.c_str());
    }
    // GET inputElevatorMaxAngleMNL value on <ESP_IP>/get?inputElevatorMaxAngleMNL=<inputMessage>
    else if (request->hasParam(PARAM_ELMM)) {
      inputMessage = request->getParam(PARAM_ELMM)->value();
      writeFile(SPIFFS, "/inputElevatorMaxAngleMNL.txt", inputMessage.c_str());
    }
    // GET inputRudderMaxAngleMNL value on <ESP_IP>/get?inputRudderMaxAngleMNL=<inputMessage>
    else if (request->hasParam(PARAM_RUMM)) {
      inputMessage = request->getParam(PARAM_RUMM)->value();
      writeFile(SPIFFS, "/inputRudderMaxAngleMNL.txt", inputMessage.c_str());
    }
    // GET inputAileroLMaxAngleAuto value on <ESP_IP>/get?inputAileroLMaxAngleAuto=<inputMessage>
    else if (request->hasParam(PARAM_ALMA)) {
      inputMessage = request->getParam(PARAM_ALMA)->value();
      writeFile(SPIFFS, "/inputAileroLMaxAngleAuto.txt", inputMessage.c_str());
    }
    // GET inputAileroRMaxAngleAuto value on <ESP_IP>/get?inputAileroRMaxAngleAuto=<inputMessage>
    else if (request->hasParam(PARAM_ARMA)) {
      inputMessage = request->getParam(PARAM_ARMA)->value();
      writeFile(SPIFFS, "/inputAileroRMaxAngleAuto.txt", inputMessage.c_str());
    }
    // GET inputElevatorMaxAngleAuto value on <ESP_IP>/get?inputElevatorMaxAngleAuto=<inputMessage>
    else if (request->hasParam(PARAM_ELMA)) {
      inputMessage = request->getParam(PARAM_ELMA)->value();
      writeFile(SPIFFS, "/inputElevatorMaxAngleAuto.txt", inputMessage.c_str());
    }
    // GET inputRudderMaxAngleAuto value on <ESP_IP>/get?inputRudderMaxAngleAuto=<inputMessage>
    else if (request->hasParam(PARAM_RUMA)) {
      inputMessage = request->getParam(PARAM_RUMA)->value();
      writeFile(SPIFFS, "/inputRudderMaxAngleAuto.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
  server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {handleDoUpdate(request, filename, index, data, len, final);}
  );
  server.onNotFound(notFound);
  server.begin();
  //Update.onProgress(printProgress);
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  //Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    //Serial.println("- empty file or failed to open file");
    return String();
  }
  //Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  //Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "inputLatitude"){
    return readFile(SPIFFS, "/inputLatitude.txt");
  }
  else if(var == "inputLongitude"){
    return readFile(SPIFFS, "/inputLongitude.txt");
  }
  else if(var == "inputAltitude"){
    return readFile(SPIFFS, "/inputAltitude.txt");
  }
  else if(var == "inputRollKp"){
    return readFile(SPIFFS, "/inputRollKp.txt");
  }
  else if(var == "inputRollKi"){
    return readFile(SPIFFS, "/inputRollKi.txt");
  }
  else if(var == "inputRollKd"){
    return readFile(SPIFFS, "/inputRollKd.txt");
  }
  else if(var == "inputYawKp"){
    return readFile(SPIFFS, "/inputYawKp.txt");
  }
  else if(var == "inputYawKi"){
    return readFile(SPIFFS, "/inputYawKi.txt");
  }
  else if(var == "inputYawKd"){
    return readFile(SPIFFS, "/inputYawKd.txt");
  }
  else if(var == "inputPitchKp"){
    return readFile(SPIFFS, "/inputPitchKp.txt");
  }
  else if(var == "inputPitchKi"){
    return readFile(SPIFFS, "/inputPitchKi.txt");
  }
  else if(var == "inputPitchKd"){
    return readFile(SPIFFS, "/inputPitchKd.txt");
  }
  else if(var == "inputBankKp"){
    return readFile(SPIFFS, "/inputBankKp.txt");
  }
  else if(var == "inputBankKi"){
    return readFile(SPIFFS, "/inputBankKi.txt");
  }
  else if(var == "inputBankKd"){
    return readFile(SPIFFS, "/inputBankKd.txt");
  }
  else if(var == "inputAltitudeKp"){
    return readFile(SPIFFS, "/inputAltitudeKp.txt");
  }
  else if(var == "inputAltitudeKi"){
    return readFile(SPIFFS, "/inputAltitudeKi.txt");
  }
  else if(var == "inputAltitudeKd"){
    return readFile(SPIFFS, "/inputAltitudeKd.txt");
  }
  else if(var == "inputAltitudeKd"){
    return readFile(SPIFFS, "/inputAltitudeKd.txt");
  }
  else if(var == "inputAltitudeKd"){
    return readFile(SPIFFS, "/inputAltitudeKd.txt");
  }
  else if(var == "inputAltitudeKd"){
    return readFile(SPIFFS, "/inputAltitudeKd.txt");
  }
  else if(var == "inputAltitudeKd"){
    return readFile(SPIFFS, "/inputAltitudeKd.txt");
  }
  else if(var == "inputAileroLzero"){
    return readFile(SPIFFS, "/inputAileroLzero.txt");
  }
  else if(var == "inputAileroRzero"){
    return readFile(SPIFFS, "/inputAileroRzero.txt");
  }
  else if(var == "inputElevatorZero"){
    return readFile(SPIFFS, "/inputElevatorZero.txt");
  }
  else if(var == "inputRudderZero"){
    return readFile(SPIFFS, "/inputRudderZero.txt");
  }
  else if(var == "inputAileroLMaxAngleMNL"){
    return readFile(SPIFFS, "/inputAileroLMaxAngleMNL.txt");
  }
  else if(var == "inputAileroRMaxAngleMNL"){
    return readFile(SPIFFS, "/inputAileroRMaxAngleMNL.txt");
  }
  else if(var == "inputElevatorMaxAngleMNL"){
    return readFile(SPIFFS, "/inputElevatorMaxAngleMNL.txt");
  }
  else if(var == "inputRudderMaxAngleMNL"){
    return readFile(SPIFFS, "/inputRudderMaxAngleMNL.txt");
  }
  else if(var == "inputAileroLMaxAngleAuto"){
    return readFile(SPIFFS, "/inputAileroLMaxAngleAuto.txt");
  }
  else if(var == "inputAileroRMaxAngleAuto"){
    return readFile(SPIFFS, "/inputAileroRMaxAngleAuto.txt");
  }
  else if(var == "inputElevatorMaxAngleAuto"){
    return readFile(SPIFFS, "/inputElevatorMaxAngleAuto.txt");
  }
  else if(var == "inputRudderMaxAngleAuto"){
    return readFile(SPIFFS, "/inputRudderMaxAngleAuto.txt");
  }
  else if(var == "mag_heading"){
    return String(mag_heading,3);
  }
  else if(var == "ground_speed"){
    return String(ground_speed,3);
  }
  else if(var == "current_lat"){
    return String(current_lat,8);
  }
  else if(var == "current_lon"){
    return String(current_lon,8);
  }
  else if(var == "current_alt"){
    return String(current_alt,3);
  }
  else if(var == "gps_hdop"){
    return String(gps_hdop,3);
  }
  else if(var == "bpm_altitude"){
    return String(bpm_altitude,3);
  }
  else if(var == "bpm_pressure"){
    return String(bpm_pressure,3);
  }
  else if(var == "bpm_temperature"){
    return String(bpm_temperature,3);
  }
  else if(var == "target_heading"){
    return String(target_heading,3);
  }
  else if(var == "target_distance"){
    return String(target_distance,3);
  }
  else{
    return "N/A";
  }
  return String();
}

void refreshUserInput(){
  
  target_lat = readFile(SPIFFS, "/inputLatitude.txt").toFloat();
  target_lon = readFile(SPIFFS, "/inputLongitude.txt").toFloat();
  target_alt = readFile(SPIFFS, "/inputAltitude.txt").toFloat();

  rollKp = readFile(SPIFFS, "/inputRollKp.txt").toFloat();
  rollKi = readFile(SPIFFS, "/inputRollKi.txt").toFloat();
  rollKd = readFile(SPIFFS, "/inputRollKd.txt").toFloat();

  yawKp = readFile(SPIFFS, "/inputYawKp.txt").toFloat();
  yawKi = readFile(SPIFFS, "/inputYawKi.txt").toFloat();
  yawKd = readFile(SPIFFS, "/inputYawKd.txt").toFloat();

  pitchKp = readFile(SPIFFS, "/inputPitchKp.txt").toFloat();
  pitchKi = readFile(SPIFFS, "/inputPitchKi.txt").toFloat();
  pitchKd = readFile(SPIFFS, "/inputPitchKd.txt").toFloat();
  
  bankKp = readFile(SPIFFS, "/inputBankKp.txt").toFloat();
  bankKi = readFile(SPIFFS, "/inputBankKi.txt").toFloat();
  bankKd = readFile(SPIFFS, "/inputBankKd.txt").toFloat();

  altitudeKp = readFile(SPIFFS, "/inputAltitudeKp.txt").toFloat();
  altitudeKi = readFile(SPIFFS, "/inputAltitudeKi.txt").toFloat();
  altitudeKi = readFile(SPIFFS, "/inputAltitudeKd.txt").toFloat();

  //int SERVO_DISPLACEMENT_ML = 30
  //int SERVO_DISPLACEMENT_PID = 30
  AILERON_L_ZERO = readFile(SPIFFS, "/inputAileroLzero.txt").toInt();
  AILERON_R_ZERO = readFile(SPIFFS, "/inputAileroRzero.txt").toInt();
  ELEVATOR_ZERO = readFile(SPIFFS, "/inputElevatorZero.txt").toInt();
  RUDDER_ZERO = readFile(SPIFFS, "/inputRudderZero.txt").toInt();
  
  AILERON_DISPLACEMENT_ML = readFile(SPIFFS, "/inputAileroLMaxAngleMNL.txt").toInt();
  //int inputAileroRMaxAngleMNL = readFile(SPIFFS, "/inputAileroRMaxAngleMNL.txt").toInt();
  ELEVATOR_DISPLACEMENT_ML = readFile(SPIFFS, "/inputElevatorMaxAngleMNL.txt").toInt();
  RUDDER_DISPLACEMENT_ML = readFile(SPIFFS, "/inputRudderMaxAngleMNL.txt").toInt();

  AILERON_DISPLACEMENT_PID = readFile(SPIFFS, "/inputAileroLMaxAngleAuto.txt").toInt();
  //int inputAileroRMaxAngleAuto = readFile(SPIFFS, "/inputAileroRMaxAngleAuto.txt").toInt();
  ELEVATOR_DISPLACEMENT_PID = readFile(SPIFFS, "/inputElevatorMaxAngleAuto.txt").toInt();
  RUDDER_DISPLACEMENT_PID = readFile(SPIFFS, "/inputRudderMaxAngleAuto.txt").toInt();
  
}

void handleUpdate(AsyncWebServerRequest *request) {
  char* html = "<h2>Firmware Update</h2><form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form><a href='http://192.168.4.1'>Back</a>";
  request->send(200, "text/html", html);
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    Serial.println("Update");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  } else {
    Serial.printf("Progress: %d%%\n", (Update.progress()*100)/Update.size());
  }

  if (final) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");  
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      Serial.println("Update complete");
      Serial.flush();
      ESP.restart();
    }
  }
}
/*
void printProgress(size_t prg, size_t sz) {
  Serial.printf("Progress: %d%%\n", (prg*100)/content_len);
}
*/
void printPID(){
  
  Serial.print("Roll: Kp=");
  Serial.print(rollKp);
  Serial.print(", Ki=");
  Serial.print(rollKi);
  Serial.print(", Kd=");
  Serial.println(rollKd);
  
  Serial.print("Yaw: Kp=");
  Serial.print(yawKp);
  Serial.print(", Ki=");
  Serial.print(yawKi);
  Serial.print(", Kd=");
  Serial.println(yawKd);
  
  Serial.print("Pitch: Kp=");
  Serial.print(pitchKp);
  Serial.print(", Ki=");
  Serial.print(pitchKi);
  Serial.print(", Kd=");
  Serial.println(pitchKd);
  
  Serial.print("Bank: Kp=");
  Serial.print(bankKp);
  Serial.print(", Ki=");
  Serial.print(bankKi);
  Serial.print(", Kd=");
  Serial.println(bankKd);
  
  Serial.print("Altitude: Kp=");
  Serial.print(altitudeKp);
  Serial.print(", Ki=");
  Serial.print(altitudeKi);
  Serial.print(", Kd=");
  Serial.println(altitudeKd);
  
}

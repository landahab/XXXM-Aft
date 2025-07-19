#include <esp_now.h>
#include <WiFi.h>


// ----------------------------
//  User Settings
// ----------------------------
// These are the setup parameters for the SSR1 on an ESP32

// Device IDs, for external reference
#define FIRMWARE_ID "SSR1-P_TCode_ESP32_Beta1.ino"  // Device and firmware version
#define TCODE_VER "TCode v0.3"  // Current version of TCode

// Pin assignments
#define Encoder_PWM_PIN 5       // PWM feedback pin (if used) - P pad on AS5048a
#define ChipSelect_PIN 10         // SPI chip select pin - CSn on AS5048a (By default on ESP32-S3: MISO = D13, MOSI = D11, CLK = D12)
#define Enable_PIN 4            // Motor enable - EN on SFOCMini
#define PWMchannel1_PIN 3       // Motor channel 1 - IN1 on SFOCMini
#define PWMchannel2_PIN 2       // Motor channel 2 - IN2 on SFOCMini
#define PWMchannel3_PIN 1       // Motor channel 3 - IN3 on SFOCMini

// Drive Parameters
#define MotorA_Voltage 20                      // Motor operating voltage (12-20V)
#define MotorA_Current 1.5                       // Maximum operating current (Amps)
// The control code needs to know the angle of the motor relative to the encoder - "Zero elec. angle".
// If a value is not entered it will perform a quick operation on startup to estimate this.
// This will be displayed in the serial monitor each time the device starts up.
// If the device is noticably faster in one direction the angle is out of alignment, try increasing or decreasing it by small increments (eg +/- 0.1).
#define MotorA_ParametersKnown true            // Once you know the zero elec angle for the motor enter it below and set this flag to true.
#define MotorA_ZeroElecAngle 5.84              // This number is the zero angle (in radians) for the motor relative to the encoder.
#define MotorA_SensorDirection Direction::CW   // Do not change. If the motor is showing CCW rotate the motor connector 180 degrees to reverse the motor.
#define SensorA_UseMT6701 true                 // Use the MT6701 encoder via SSI rather than the default AS5048a encoder via SPI.
#define SensorA_UsePWM false                   // SPI feedback on the AS5048a is default because it's a lot smoother and quieter! Change this to true if you want to use PWM feedback.

// Control constants
// (a.k.a. magic numbers for Eve)
#define RAIL_LENGTH 125           // Physical maximum travel of the receiver (mm)
#define STROKE_LENGTH 120         // Operating stroke length (mm)
#define PULLEY_CIRCUMFERENCE 3  // Drive pulley circumference (mm)
#define P_CONST 0.002             // Motor PID proportional constant
#define LOW_PASS 0.8              // Low pass filter factor for static noise reduction ( number < 1, 0 = none)
// Derived constants
#define ANG_TO_POS (10000*PULLEY_CIRCUMFERENCE)/(2*3.14159*STROKE_LENGTH) // Number to convert a motor angle to a 0-10000 axis position
#define START_OFFSET 2*3.14159*(RAIL_LENGTH-STROKE_LENGTH)/(2*PULLEY_CIRCUMFERENCE)  // Offset angle from endstop on startup (rad)

// Other functions
#define MIN_SMOOTH_INTERVAL 3     // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL 100   // Maximum auto-smooth ramp interval for live commands (ms)

// T-Code Channels
#define CHANNELS 3                // Number of channels of each type (LRVA)

// Libraries used
#include <EEPROM.h> // Permanent memory
#include <SimpleFOC.h> // Motor controller code
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"





#define MaxGearChangeThrottle 30
int safeGearState = 0;

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above


// encoder position monitor variables
volatile int encoderPulseLength = 464;
volatile int encoderPulseCycle = 920;
volatile int encoderPulseStart = 0;
// range is 5-928
volatile int longest = 500;
volatile int shortest = 500;

// Declare an SSI, PWM and SPI sensor. Only one will be used.
MagneticSensorMT6701SSI sensorA = MagneticSensorMT6701SSI(ChipSelect_PIN);
MagneticSensorPWM sensorA1 = MagneticSensorPWM(Encoder_PWM_PIN, 5, 928);
MagneticSensorSPI sensorA2 = MagneticSensorSPI(ChipSelect_PIN, 14, 0x3FFF);
// BLDC motor & driver instance
BLDCMotor motorA = BLDCMotor(11,11.1);
BLDCDriver3PWM driverA = BLDCDriver3PWM(PWMchannel1_PIN, PWMchannel2_PIN, PWMchannel3_PIN, Enable_PIN);

#define GearServo_PIN 7
#define GearServo_MIN 1000
#define GearServo_MAX 2000
#define GearServo_ZERO (ThrottleServo_MIN + ThrottleServo_MAX)/2

#define ThrottleServo_PIN 8
#define ThrottleServo_MIN 1000
#define ThrottleServo_MAX 2000
#define ThrottleServo_ZERO (ThrottleServo_MIN + ThrottleServo_MAX)/2

#define BilgeRelay_PIN 6

#define PWM_RESOLUTION_BITS 16
#define DS3225_Servo_Freq 330

// Position variables
int xLin;
float zeroAngle;
float xPosition;
float mode;
long startTime;
float motorVoltage;


class Servo {
  public:
    Servo(){}
    Servo(int pin, int freq, int pwm_bits, int min, int max, int zero = -1){
      _pin = pin;
      _freq = freq;
      _pwm_bits = pwm_bits; //resolution in bits
      _servo_int = 1000000/freq;
      _min = min;
      _max = max;
      if(zero<0){
        _zero = (min + max) / 2;
      }
      else{
        _zero = zero;
      }

      // Initialize the servo pin
      pinMode(_pin, OUTPUT);

      ledcAttach(_pin, _freq, _pwm_bits);
    }
    void write(float ratio) {
      int writeValue;
      // Ensure ratio is within bounds
      if (ratio < 0.0f) {
        writeValue=_min;
      } else if (ratio > 1.0f) {
        writeValue = _max;
      }
      else{
        writeValue = static_cast<int>(ratio * (_max - _min) + _min);
      }
    
      ledcWrite(_pin, map(writeValue, 0, _servo_int, 0, 65535));
    }
  private:    
    int _pin;
    int _freq;
    int _pwm_bits;
    int _servo_int;
    int _min;
    int _max;
    int _zero;
};


Servo GearServo;
Servo ThrottleServo;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float steer; //-10000 to 10000
  float throttle; //0 to 10000
  int gear; //-1,0,1
  int bilge; //1,0
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message foreBoard;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
 // Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  foreBoard.steer = myData.steer;
  foreBoard.throttle = myData.throttle;
  foreBoard.gear = myData.gear;
  foreBoard.bilge = myData.bilge;

  // Serial.printf("steer value: %d \n", foreBoard.steer);
  // Serial.printf("throttle value: %d \n", foreBoard.throttle);
  // Serial.printf("gear value: %d \n", foreBoard.gear);
  // Serial.printf("bilge value: %d \n", foreBoard.bilge);
  // Serial.println();
}
 
void setup() {

  // Start serial connection and report status
  Serial.begin(115200);

  // #ESP32# Enable EEPROM
  EEPROM.begin(320);

  // Set Starting state
  zeroAngle = 0;
  mode = 0;

  GearServo=Servo(GearServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, GearServo_MIN, GearServo_MAX);
  ThrottleServo= Servo(ThrottleServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, ThrottleServo_MIN, ThrottleServo_MAX);
  pinMode(BilgeRelay_PIN, OUTPUT);

  // initialise encoder hardware
  if (SensorA_UseMT6701) {
    sensorA.init();
  } else if (SensorA_UsePWM) {
    sensorA1.init();
  } else {
    sensorA2.init();
  }
  
  // driver config
  // power supply voltage [V]
  driverA.voltage_power_supply = MotorA_Voltage;
  // Max DC voltage allowed - default voltage_power_supply
  driverA.voltage_limit = 20;
  // driver init
  driverA.init();

  // limiting motor movements
  motorA.current_limit = MotorA_Current;   // [Amps]

  // set control loop type to be used
  motorA.torque_controller = TorqueControlType::voltage;
  motorA.controller = MotionControlType::torque;

  // link the motor to the sensor
  if (SensorA_UsePWM) { motorA.linkSensor(&sensorA1); } else { motorA.linkSensor(&sensorA); }
  // link the motor and the driver
  motorA.linkDriver(&driverA);

  // initialize motor
  motorA.init();
  if(MotorA_ParametersKnown) {
    motorA.sensor_direction = MotorA_SensorDirection;
    motorA.zero_electric_angle = MotorA_ZeroElecAngle; // rad
  } else {
    motorA.useMonitoring(Serial);
  }
  motorA.initFOC();
  
  // Set sensor angle and pre-set zero angle to current angle
  if (SensorA_UseMT6701) {
    sensorA.update();
    zeroAngle = sensorA.getAngle();
  } else if (SensorA_UsePWM) {
    sensorA1.update();
    zeroAngle = sensorA1.getAngle();
  } else {
    sensorA2.update();
    zeroAngle = sensorA2.getAngle();
  }
  // Record start time
  startTime = millis();


  foreBoard.gear = 0;
  foreBoard.steer = 0;
  foreBoard.throttle = 0;
  foreBoard.bilge = 0;

  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}



// -----------------------------
// Class to handle each axis
// -----------------------------
class Axis {

  public:
  // Setup function
  Axis() {

    // Set default dynamic parameters
    rampStartTime = 0;
    rampStart = 5000;
    rampStopTime = rampStart;
    rampStop = rampStart;

    // Set Empty Name
    Name = "";
    lastT = 0;

    // Live command auto-smooth
    minInterval = MAX_SMOOTH_INTERVAL;
      
  }

  // Function to set the axis dynamic parameters
  void Set(int x, char ext, long y) {
    unsigned long t = millis(); // This is the time now
    x = constrain(x,0,9999);
    y = constrain(y,0,9999999);
    // Set ramp parameters, based on inputs
    // Live command
    if ( y == 0 || ( ext != 'S' && ext != 'I' ) ) {
      // update auto-smooth regulator
      int lastInterval = t - rampStartTime;
      if ( lastInterval > minInterval && minInterval < MAX_SMOOTH_INTERVAL ) { minInterval += 1; }
      else if ( lastInterval < minInterval && minInterval > MIN_SMOOTH_INTERVAL ) { minInterval -= 1; } 
      // Set ramp parameters
      rampStart = GetPosition();
      rampStopTime = t + minInterval;  
    } 
    // Speed command
    else if ( ext == 'S' ) {
      rampStart = GetPosition();  // Start from current position
      int d = x - rampStart;  // Distance to move
      if (d<0) { d = -d; }
      long dt = d;  // Time interval (time = dist/speed)
      dt *= 100;
      dt /= y; 
      rampStopTime = t + dt;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    // Interval command
    else if ( ext == 'I' ) {
      rampStart = GetPosition();  // Start from current position
      rampStopTime = t + y;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    rampStartTime = t;
    rampStop = x;
    lastT = t;
  }

  // Function to return the current position of this axis
  int GetPosition() {
    int x; // This is the current axis position, 0-9999
    unsigned long t = millis(); 
    if (t > rampStopTime) {
      x = rampStop;
    } else if (t > rampStartTime) { 
      x = map(t,rampStartTime,rampStopTime,rampStart,rampStop);
    } else {
      x = rampStart;
    }
    x = constrain(x,0,9999);
    return x;
  }

  // Function to stop axis movement at current position
  void Stop() {
    unsigned long t = millis(); // This is the time now
    rampStart = GetPosition();
    rampStartTime = t;
    rampStop = rampStart;
    rampStopTime = t;
  }

  // Public variables
  String Name;  // Function name of this axis
  unsigned long lastT;  //

  private:
  
  // Movement positions
  int rampStart;
  unsigned long rampStartTime;
  int rampStop;
  unsigned long rampStopTime;

  // Live command auto-smooth regulator
  int minInterval;

};







// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop() {

  Serial.printf("steer value: %d \n", foreBoard.steer);
  Serial.printf("throttle value: %d \n", foreBoard.throttle);
  Serial.printf("gear value: %d \n", foreBoard.gear);
  Serial.printf("bilge value: %d \n", foreBoard.bilge);
  Serial.println();

    if(foreBoard.gear != safeGearState) {
    if(foreBoard.gear==0 || foreBoard.throttle <= MaxGearChangeThrottle){
      safeGearState = foreBoard.gear;
    }
  }
  if(foreBoard.bilge == 1){
          Serial.println("BILGE UP");
      delay(1000);
    digitalWrite(BilgeRelay_PIN, HIGH);
  }
    if(foreBoard.bilge == 0){
            Serial.println("BILGE DOWN");
      delay(1000);
    digitalWrite(BilgeRelay_PIN, LOW);
  }

  GearServo.write(map(foreBoard.gear, -1, 1, GearServo_MIN, GearServo_MAX));
  ThrottleServo.write(map(foreBoard.throttle, 0, 10000, ThrottleServo_MIN, ThrottleServo_MAX));

  // Run motor FOC loop                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  motorA.loopFOC();

  // Collect inputs
  // These functions query the t-code object for the position/level at a specified time
  // Number recieved will be an integer, 0-9999
  xLin = foreBoard.steer;

  // Update sensor position
  float angle;
  if (SensorA_UseMT6701) {
    sensorA.update();
    angle = sensorA.getAngle();
  } else if (SensorA_UsePWM) {
    sensorA1.update();
    angle = sensorA1.getAngle();
  } else {
    sensorA2.update();
    angle = sensorA2.getAngle();
  }
  // Determine the linear position of the receiver in (0-10000)
  xPosition = (angle - zeroAngle)*ANG_TO_POS; 

  // Control by motor voltage
  float motorVoltageNew;
  // If the device is in startup mode roll downwards for two seconds and press against bottom stop.
  // Distance of travel is 12,000 (>10,000) just to make sure that the receiver reaches the bottom.
  if (mode == 0) {
    xLin  = map(millis()-startTime,0,2000,0,-12000);
    if (millis() > (startTime + 2000)) {
      mode = 1;
      zeroAngle = angle + START_OFFSET;
    }
    motorVoltageNew = P_CONST*(xLin - xPosition);
    if (motorVoltageNew < -0.5) { motorVoltageNew = -0.5; }
  // Otherwise set motor voltage based on position error     
  } else {
    motorVoltageNew = P_CONST*(xLin - xPosition);
  }
  // Low pass filter to reduce motor noise
  motorVoltage = LOW_PASS*motorVoltage + (1-LOW_PASS)*motorVoltageNew;  
  // Motion control function
  motorA.move(motorVoltage);
  delay(1);
}

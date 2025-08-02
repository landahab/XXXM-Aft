#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define MaxGearChangeThrottle 30
int safeGearState = 0;

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above

#define GearServo_PIN 7
#define GearServo_REVERSE 500
#define GearServo_NEUTRAL 1000
#define GearServo_FORWARD 1500

#define ThrottleServo_PIN 8
#define ThrottleServo_MIN 1000
#define ThrottleServo_MAX 2000
#define ThrottleServo_ZERO (ThrottleServo_MIN + ThrottleServo_MAX) / 2

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

enum GEAR_STATE {
  GEAR_STATE_REVERSE = -1,
  GEAR_STATE_NEUTRAL = 0,
  GEAR_STATE_FORWARD = 1,
};

class Servo
{
public:
  Servo() {}
  Servo(int pin, int freq, int pwm_bits, int min, int max, int zero = -1)
  {
    Serial.printf("Instantiating servo on pin %d\n", pin);

    _pin = pin;
    _freq = freq;
    _pwm_bits = pwm_bits; // resolution in bits
    _servo_int = 1000000 / freq;
    _min = min;
    _max = max;
    if (zero < 0)
    {
      _zero = (min + max) / 2;
    }
    else
    {
      _zero = zero;
    }

    // Initialize the servo pin
    pinMode(_pin, OUTPUT);

    ledcAttach(_pin, _freq, _pwm_bits);
  }

  void write(float ratio)
  {
    int writeValue;
    // Ensure ratio is within bounds
    if (ratio < 0.0f)
    {
      writeValue = _min;
    }
    else if (ratio > 1.0f)
    {
      writeValue = _max;
    }
    else
    {
      writeValue = static_cast<int>(ratio * (_max - _min) + _min);
    }

    ledcWrite(_pin, map(writeValue, 0, _servo_int, 0, 65535));
  }

private:
  void init(int pin, int freq, int pwm_bits, int min, int max, int zero = -1) {
    
  }

  int _pin;
  int _freq;
  int _pwm_bits;
  int _servo_int = 0;
  int _min;
  int _max;
  int _zero;
};

Servo GearServo(GearServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, GearServo_REVERSE, GearServo_FORWARD);
Servo ThrottleServo(ThrottleServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, ThrottleServo_MIN, ThrottleServo_MAX);

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int id;
  int steer;    //-10000 to 10000
  int throttle; // 0 to 10000
  GEAR_STATE gear;     //-1,0,1
  int bilge;    // 1,0
} struct_message;

// Create a structure to hold the readings from each board
struct_message foreBoard;

void dumpState(struct_message *msg) {
  Serial.printf("=============\n");
  Serial.printf("ID:        %d\n", msg->id);
  Serial.printf("Steer:     %d\n", msg->steer);
  Serial.printf("Throttle:  %d\n", msg->throttle);
  Serial.printf("Gear       %d\n", (int)msg->gear);
  Serial.printf("Bilge:     %d\n", msg->bilge);
  Serial.printf("=============\n");
  Serial.println();
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  struct_message incomingMessage = {0};
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.printf("Received message from %s\n", macStr);

  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));

  Serial.println("Incoming message:");
  dumpState(&incomingMessage);

  // Update the structures with the new incoming data
  foreBoard.steer = incomingMessage.steer;
  foreBoard.throttle = incomingMessage.throttle;
  foreBoard.gear = incomingMessage.gear;
  foreBoard.bilge = incomingMessage.bilge;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Setting up...");

  Serial.println("Initialising state");
  foreBoard.gear = GEAR_STATE_NEUTRAL;
  foreBoard.steer = 0;
  foreBoard.throttle = 0;
  foreBoard.bilge = 0;

  Serial.println("Setting pin modes");
  pinMode(GearServo_PIN, OUTPUT);
  pinMode(ThrottleServo_PIN, OUTPUT);
  pinMode(BilgeRelay_PIN, OUTPUT);

  Serial.println("Configuring WiFi");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("My MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Done");
}

// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop()
{
  // If the gear selected on the control panel is different from the currently selected gear
  if (foreBoard.gear != safeGearState)
  {
    // Check that it's actually safe to change gear right now, i.e. we're changing INTO neutral, or the throttle is low
    if (foreBoard.gear == 0 || foreBoard.throttle <= MaxGearChangeThrottle)
    {
      // It's safe, let's change it
      Serial.printf("Changing gear from %d to %d\n", safeGearState, foreBoard.gear);
      safeGearState = foreBoard.gear;
    }
  }

  if (foreBoard.bilge == 1)
  {
    delay(1000);
    digitalWrite(BilgeRelay_PIN, HIGH);
  } 
  else if (foreBoard.bilge == 0)
  {
    delay(1000);
    digitalWrite(BilgeRelay_PIN, LOW);
  }

  switch (safeGearState)
  {
  case GEAR_STATE_REVERSE:
    GearServo.write(GearServo_REVERSE);
  case GEAR_STATE_NEUTRAL:
    GearServo.write(GearServo_NEUTRAL);
  case GEAR_STATE_FORWARD:
    GearServo.write(GearServo_FORWARD);
  default:
    break;
  }

  ThrottleServo.write(map(foreBoard.throttle, 0, 10000, ThrottleServo_MIN, ThrottleServo_MAX));

  delay(1);
}

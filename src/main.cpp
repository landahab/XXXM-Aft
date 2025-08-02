#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define MaxGearChangeThrottle 30

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above

#define GearServo_PIN 9
#define GearServo_REVERSE 2000 
#define GearServo_NEUTRAL 2300
#define GearServo_FORWARD 2600

#define ThrottleServo_PIN 10
#define ThrottleServo_MIN 2000
#define ThrottleServo_MAX 2500
#define ThrottleServo_ZERO (ThrottleServo_MIN + ThrottleServo_MAX) / 2

#define THROTTLE_INPUT_MAX 84000

#define BilgeRelay_PIN 6

#define PWM_RESOLUTION_BITS 12
#define DS3225_Servo_Freq 330

uint8_t newMACAddress[] = {0xb4,0x3a,0x45,0xb0,0xd4,0xa8};

// Position variables
int xLin;
float zeroAngle;
float xPosition;
float mode;
long startTime;
float motorVoltage;
uint64_t cycle = 0;
uint64_t pktsReceived = 0;

enum FOREBOARD_GEAR_STATE {
  GEAR_STATE_REVERSE = -1,
  GEAR_STATE_NEUTRAL = 0,
  GEAR_STATE_FORWARD = 1
};
FOREBOARD_GEAR_STATE safeGearState = GEAR_STATE_NEUTRAL;


class Servo
{
public:
  Servo() {}
  
  Servo(int pin, int freq, int pwm_bits, int in_min, int in_max, int out_min, int out_max)//, int zero = -1)
  {
    Serial.printf("Instantiating servo on pin %d\n", pin);

    _pin = pin;
    _freq = freq;
    _pwm_bits = pwm_bits; // resolution in bits
    _servo_int = 1000000 / freq;
    _in_min = in_min;
    _in_max = in_max;
    _out_min = out_min;
    _out_max = out_max;

    // Initialize the servo pin
    ledcAttach(_pin, _freq, _pwm_bits);
  }

  void write(int value)
  {
    int clamped;
    if (value > _in_max) {
      clamped = _in_max;
    } else if (value < _in_min) {
      clamped = _in_min;
    } else {
      clamped = value;
    }
    long out = map(clamped, _in_min, _in_max, _out_min, _out_max);
    ledcWrite(_pin, out);
  }

private:
  int _pin;
  int _freq;
  int _pwm_bits;
  int _servo_int;
  int _in_min;
  int _in_max;
  int _out_min;
  int _out_max;
  int _zero;
};

Servo GearServo;
Servo ThrottleServo;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int id;
  int steer;    //-10000 to 10000
  int throttle; // 0 to 10000
  FOREBOARD_GEAR_STATE gear;     //-1,0,1
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

  // Update the structures with the new incoming data
  foreBoard.steer = incomingMessage.steer;
  foreBoard.throttle = incomingMessage.throttle;
  foreBoard.gear = incomingMessage.gear;
  foreBoard.bilge = incomingMessage.bilge;

  pktsReceived++;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("Setting up...");

  Serial.println("Initialising state");
  foreBoard.gear = GEAR_STATE_NEUTRAL;
  foreBoard.steer = 0;
  foreBoard.throttle = 0;
  foreBoard.bilge = 0;

  Serial.println("Setting pin modes");
  pinMode(BilgeRelay_PIN, OUTPUT);

  Serial.println("Instantiating servos");
  GearServo = Servo(GearServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, 0, 2, GearServo_REVERSE, GearServo_FORWARD);
  ThrottleServo = Servo(ThrottleServo_PIN, DS3225_Servo_Freq, PWM_RESOLUTION_BITS, 0, 84000, ThrottleServo_MIN, ThrottleServo_MAX);

  Serial.println("Configuring WiFi");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Change our MAC to the one we want
  esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  if (err != ESP_OK) {
    Serial.println("Failed to change MAC address!");
  }

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

int gearStateToServoInput(FOREBOARD_GEAR_STATE gs) {
  // This maps from the range received from the foreboard (-1, 0, 1) to values in the range
  // which makes the servo behave appropriately.

  switch (gs) {
  case GEAR_STATE_REVERSE:
    return 2;
  case GEAR_STATE_NEUTRAL:
    return 1;
  case GEAR_STATE_FORWARD:
    return 0;
  default:
    Serial.printf("Can't map invalid gear state: %d\n", gs);
    return 1;
  }
}

int throttleStateToServoInput(int throttle) {
  // Need to invert the throttle signal so the throttle servo moves in the correct direction
  if (throttle > THROTTLE_INPUT_MAX) {
    throttle = THROTTLE_INPUT_MAX;
  }
  return THROTTLE_INPUT_MAX - throttle;
}

// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop()
{
  if (cycle++ % 500 == 0) {
    dumpState(&foreBoard);
    Serial.printf("Packets received in last period: %lld\n", pktsReceived);
    pktsReceived = 0;
  }

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
    digitalWrite(BilgeRelay_PIN, HIGH);
  } 
  else if (foreBoard.bilge == 0)
  {
    digitalWrite(BilgeRelay_PIN, LOW);
  }
  
  GearServo.write(gearStateToServoInput(safeGearState));
  ThrottleServo.write(throttleStateToServoInput(foreBoard.throttle));
  delay(1);
}
 
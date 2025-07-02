#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"
#include <Adafruit_Fingerprint.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include <SoftwareSerial.h>

BluetoothSerial SerialBT;

// === LCD ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Fingerprint ===
#define RX_PIN 16
#define TX_PIN 17
HardwareSerial mySerial(2);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// === Servo ===
Servo gateServo;
#define SERVO_PIN 19

// === Buzzer & Bluetooth Indicator ===
#define BUZZER_PIN 4
const int blub = 15;

// === Keypad ===
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 13, 12, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
String enteredPassword = "";
String correctPassword = "4238";

// === SIM800L ===
SoftwareSerial sim800(18, 5);  // TX, RX

// === Sensors & Pump ===
const int fireSensorPin = 34;
const int gasSensorPin = 35;
const int pump = 23;

// === Bluetooth status flag ===
bool isBTConnected = false;

// === FreeRTOS Dummy Task ===
void oneSecondTask(void* parameter) {
  for (;;) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  SerialBT.begin("smart_home");  // ✅ Bluetooth name
  Serial.begin(115200);
  mySerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
  sim800.begin(9600);

  finger.begin(57600);
  if (!finger.verifyPassword()) {
    Serial.println("Fingerprint sensor not found!");
  } else {
    Serial.println("Fingerprint sensor found.");
  }

  pinMode(blub, OUTPUT);
  digitalWrite(blub, LOW);

  Wire.begin(21, 22);
  lcd.begin(16, 2);
  lcd.backlight();
  showWelcomeMessage();

  gateServo.attach(SERVO_PIN);
  gateServo.write(0);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(pump, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(pump, LOW);

  sim800.println("AT");
  delay(500);
  sim800.println("AT+CMGF=1");
  delay(500);

  xTaskCreatePinnedToCore(oneSecondTask, "OneSecondTask", 1000, NULL, 1, NULL, 1);
}

void loop() {
  // ✅ Bluetooth Connected/Disconnected Message
  if (SerialBT.hasClient()) {
    if (!isBTConnected) {
      isBTConnected = true;
      Serial.println("✅ One device connected!");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Bluetooth");
      lcd.setCursor(0, 1);
      lcd.print("Connected");

      beepBuzzer(100, 2);
      delay(2000);
      showWelcomeMessage();
    }
  } else {
    if (isBTConnected) {
      isBTConnected = false;
      Serial.println("❌ Device disconnected.");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Bluetooth");
      lcd.setCursor(0, 1);
      lcd.print("Disconnected");

      delay(2000);
      showWelcomeMessage();
    }
  }

  // Bluetooth control via app
  if (SerialBT.available()) {
    char received = SerialBT.read();
    Serial.print("Received: ");
    Serial.println(received);
    if (received == 'B') digitalWrite(blub, HIGH);
    else if (received == 'b') digitalWrite(blub, LOW);
    beepBuzzer(100, 1);
  }

  // Fire & Gas Detection
  int fireValue = analogRead(fireSensorPin);
  int gasValue = analogRead(gasSensorPin);
  bool fireDetected = fireValue < 1000;
  bool gasDetected = gasValue > 350;

  if (fireDetected) {
    handleEmergency("Fire", fireValue, "Fire detected! Take action.");
  } else if (gasDetected) {
    handleEmergency("Gas", gasValue, "Gas leak detected!");
  } else {
    digitalWrite(pump, LOW);
  }

  // Fingerprint
  uint8_t id = getFingerprintID();
  if (id > 0) {
    grantAccess("FP ID: " + String(id));
  }

  // Keypad Input
  char key = keypad.getKey();
  if (key) handleKeypadInput(key);
}

void showWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for ID...");
  lcd.setCursor(2, 1);
  lcd.print("or Password");
}

void handleEmergency(String type, int value, String smsText) {
  digitalWrite(pump, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(type + ":");
  lcd.print(value);
  lcd.setCursor(0, 1);
  lcd.print(type + " Detected");
  beepBuzzer(100, 5);
  delay(3000);
  digitalWrite(pump, LOW);
  showWelcomeMessage();
  delay(100);
  sendSMS(smsText);
  delay(100);
}

void handleKeypadInput(char key) {
  if (key == '#') {
    if (enteredPassword == correctPassword) {
      grantAccess("Correct Password");
    } else {
      wrongPasswordAlert();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wrong Password");
      Serial.println("Wrong Password");
      delay(2000);
      showWelcomeMessage();
    }
    enteredPassword = "";
  } else if (key == '*') {
    enteredPassword = "";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cleared");
    delay(1000);
    showWelcomeMessage();
  } else {
    if (enteredPassword.length() < 16) {
      enteredPassword += key;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Password:");
      lcd.setCursor(0, 1);
      for (int i = 0; i < enteredPassword.length(); i++) {
        lcd.print("*");
      }
    }
  }
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return 0;
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return 0;
  p = finger.fingerSearch();
  if (p != FINGERPRINT_OK) return 0;
  return finger.fingerID;
}

void grantAccess(String source) {
  Serial.println(source);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Access Granted");
  lcd.setCursor(0, 1);
  lcd.print(source);
  beepBuzzer(100, 1);
  openServoSlowly();
  delay(3000);
  closeServoSlowly();
  showWelcomeMessage();
}

void openServoSlowly() {
  for (int pos = 0; pos <= 70; pos++) {
    gateServo.write(pos);
    delay(10);
  }
}

void closeServoSlowly() {
  for (int pos = 70; pos >= 0; pos--) {
    gateServo.write(pos);
    delay(10);
  }
}

void beepBuzzer(int duration, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

void wrongPasswordAlert() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void sendSMS(String message) {
  sim800.println("AT+CMGF=1");
  delay(500);
  sim800.println("AT+CMGS=\"01893313112\"");  // Change this number if needed
  delay(500);
  sim800.print(message);
  delay(500);
  sim800.write(26);  // Ctrl+Z
  delay(5000);
}
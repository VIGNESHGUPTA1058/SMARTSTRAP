# SMARTSTRAP
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;

// Serial ports
HardwareSerial gpsSerial(1);
HardwareSerial gsmSerial(2);

// Phone number (CHANGE THIS)
String emergencyNumber = "+91XXXXXXXXXX";

// Variables
long duration;
int distance;

void setup() {
  pinMode(SOS_BUTTON, INPUT_PULLUP);
  pinMode(VIBRATION_MOTOR, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);

  delay(3000);
  Serial.println("Smart Safety Bag Initialized");
}

void loop() {
  // Read GPS continuously
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Proximity detection
  distance = getDistance();
  if (distance > 0 && distance < 200) { // ~2 meters
    digitalWrite(VIBRATION_MOTOR, HIGH);
  } else {
    digitalWrite(VIBRATION_MOTOR, LOW);
  }

  // SOS Button Pressed
  if (digitalRead(SOS_BUTTON) == LOW) {
    sendEmergencySMS();
    delay(10000); // avoid spamming
  }
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}

void sendEmergencySMS() {
  String message = "EMERGENCY! I need help.";

  if (gps.location.isValid()) {
    message += " Location: https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
  } else {
    message += " Location unavailable.";
  }

  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  gsmSerial.println("AT+CMGS=\"" + emergencyNumber + "\"");
  delay(1000);
  gsmSerial.print(message);
  delay(500);
  gsmSerial.write(26); // CTRL+Z
  delay(3000);

  Serial.println("Emergency SMS Sent");
}

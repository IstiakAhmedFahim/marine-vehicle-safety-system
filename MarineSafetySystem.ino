/*
 * Marine Vehicle Safety System (Thesis Project)
 * Hardware: Arduino UNO, L298N, MPU6050, HC-SR04, Water Sensor, HM-10
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Hardware Pinout
#define PIN_PWM     10
#define PIN_IN1     12
#define PIN_IN2     11
#define PIN_SERVO   13
#define PIN_TRIG    8
#define PIN_ECHO    9
#define PIN_WATER   A0
#define PIN_BT_RX   2
#define PIN_BT_TX   3

// Status Indicators
#define PIN_SAFE    4  
#define PIN_WARN    5  
#define PIN_CRIT    6  
#define PIN_BUZZ    7

// System Constants (Calibrated)
#define DIST_LIMIT  100   // cm (Static test result)
#define TILT_MAX    25    // degrees
#define LOAD_MAX    650   // analog value (~5cm)

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo rudder;
SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);

const int MPU_ADDR = 0x68;
int16_t raw_y, raw_z;
float tilt = 0.0;
int dist = 0;
int load_val = 0;

bool lockout = false;
unsigned long timer_lcd = 0;
unsigned long timer_avoid = 0;
int avoid_state = 0; 

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_SAFE, OUTPUT);
  pinMode(PIN_WARN, OUTPUT);
  pinMode(PIN_CRIT, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  rudder.attach(PIN_SERVO);
  rudder.write(90); // Center

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("System Active");
  delay(1000); 
  lcd.clear();
}

void loop() {
  read_sensors();
  check_safety();

  if (lockout) {
    stop_motor();
  } 
  else {
    // Autonomous priority
    if (dist > 0 && dist < DIST_LIMIT) {
      run_avoidance();
    } else {
      avoid_state = 0; 
      run_bt_control();
    }
  }

  // UI Update (5Hz)
  if (millis() - timer_lcd > 200) {
    update_ui();
    timer_lcd = millis();
  }
}

void read_sensors() {
  // MPU6050 Tilt
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  Wire.read(); Wire.read(); // Skip X
  raw_y = Wire.read()<<8 | Wire.read();
  raw_z = Wire.read()<<8 | Wire.read();
  tilt = atan2(raw_y, raw_z) * 57.296;

  // Water Load
  load_val = analogRead(PIN_WATER);

  // Sonar
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  dist = pulseIn(PIN_ECHO, HIGH) * 0.034 / 2;
  
  // Serial.print(tilt); Serial.print(" "); Serial.println(dist); // Debug
}

void check_safety() {
  digitalWrite(PIN_SAFE, LOW);
  digitalWrite(PIN_WARN, LOW);
  digitalWrite(PIN_CRIT, LOW);
  digitalWrite(PIN_BUZZ, LOW);
  
  lockout = false;

  // Water thresholds per thesis data
  if (load_val >= LOAD_MAX) {
    digitalWrite(PIN_CRIT, HIGH);
    digitalWrite(PIN_BUZZ, HIGH);
    lockout = true;
  } 
  else if (load_val >= 625) digitalWrite(PIN_CRIT, HIGH);
  else if (load_val >= 600) digitalWrite(PIN_WARN, HIGH);
  else if (load_val >= 100) digitalWrite(PIN_SAFE, HIGH);

  if (abs(tilt) > TILT_MAX) {
    lockout = true;
  }
}

void run_avoidance() {
  unsigned long now = millis();

  switch(avoid_state) {
    case 0: 
      stop_motor();
      timer_avoid = now;
      avoid_state = 1;
      break;
      
    case 1: // Wait 500ms
      if (now - timer_avoid > 500) {
        rudder.write(135); // Turn Right
        timer_avoid = now;
        avoid_state = 2;
      }
      break;
      
    case 2: // Move 1.5s
      motor_fwd();
      if (now - timer_avoid > 1500) {
        rudder.write(90); 
        timer_avoid = now;
        avoid_state = 3;
      }
      break;
      
    case 3: 
      motor_fwd();
      break;
  }
}

void run_bt_control() {
  if (bt.available()) {
    char cmd = bt.read();
    switch(cmd) {
      case 'F': motor_fwd(); break;
      case 'B': motor_rev(); break;
      case 'S': stop_motor(); break;
      case 'L': rudder.write(45); break;
      case 'R': rudder.write(135); break;
      case 'C': rudder.write(90); break;
    }
  }
}

void motor_fwd() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 200); // ~2000 RPM
}

void motor_rev() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_PWM, 200);
}

void stop_motor() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 0);
}

void update_ui() {
  lcd.setCursor(0, 0);
  if (abs(tilt) > TILT_MAX) lcd.print("Tilt: DANGER!   ");
  else lcd.print("Tilt: STABLE    ");

  lcd.setCursor(0, 1);
  if (load_val >= LOAD_MAX) lcd.print("Load: OVERWEIGHT");
  else if (dist < DIST_LIMIT) lcd.print("Nav:  AVOIDING  ");
  else lcd.print("Sys:  OK        ");
}
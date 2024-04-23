#include <Arduino.h>

const char MOTOR_ENA = 6;
const char MOTOR_IN1 = 7;
const char MOTOR_IN2 = 8;

const char ENCODER = A7;

int rotation = 0;
int revolutions = 0;
int lastVal;

bool dir = false;
int speed = 120;

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(ENCODER, INPUT);
  lastVal = analogRead(ENCODER);
  Serial.begin(9600);


  
  analogWrite(MOTOR_ENA, speed);
}

void readPosition() {
  int currVal = analogRead(ENCODER);
  int diff = lastVal - currVal;
  if (diff < -500) {
    revolutions++;
  }
  else if (diff > 500) {
    revolutions--;
  }
  Serial.println(revolutions);
  lastVal = currVal;
}

void loop() {
  readPosition();
  analogWrite(MOTOR_ENA, speed);
  digitalWrite(MOTOR_IN1, dir);
  digitalWrite(MOTOR_IN2, !dir);
  if (revolutions > 1) {
    if (!dir) {
      digitalWrite(MOTOR_ENA, LOW);
      delay(100);
    }
    dir = true;
  } else if (revolutions < -1) {
    if (dir) {
      digitalWrite(MOTOR_ENA, LOW);
      delay(100);
    }
    dir = false;
  }
  
}
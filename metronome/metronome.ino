// BID Hapkit setup example 

#include <Arduino.h>

// assign IN1_pin and IN2_pin to D7 and D8
const char IN1_pin = 7;
const char IN2_pin = 8;

// assign ENA_pin to D6 which supports PWM output
// for more information check　https://www.teachmemicro.com/arduino-nano-pinout-diagram/
const char ENA_pin = 6;
bool speed = 30;
bool dir;
void setup() {
  // put your setup code here, to run once:
  
  // TODO1: set IN1_pin, IN2_pin, ENA_pin as ann OUTPUT mode
  pinMode(IN1_pin, OUTPUT);
  pinMode(IN2_pin, OUTPUT);
  pinMode(ENA_pin, OUTPUT);
  dir = true;
  }


void loop() {
  // put your main code here, to run repeatedly:

  // TODO2: drive the motor forward using IN1_pin, IN2_pin, with digitalWrite(),
  // .      and set the driving power using ENA_pin with analogWrite() with PWM value 0-255.
  //        Start with smaller PWM value for test.
  
  digitalWrite(IN1_pin, dir);
  digitalWrite(IN2_pin, !dir);
  analogWrite(ENA_pin, speed);
  delay(100);
  dir = !dir;
}
/*

int encoderPin = A7;
int ledPin = LED_BUILTIN;
int rotation = 0;
int lastVal;
int revolutions = 0;
void setup() {
  // put your setup code here, to run once:

  pinMode(ledPin, OUTPUT);
  pinMode(encoderPin, INPUT);

  Serial.begin(9600);

  int lastVal = analogRead(encoderPin);
}

void loop() {
  int currVal = analogRead(encoderPin);
  int diff = lastVal - currVal;
  if (diff > 500) {
    revolutions++;
  }
  else if (diff < -500) {
    revolutions--;
  }
  Serial.println(revolutions);
  lastVal = currVal;
  delay(10);

}


*/
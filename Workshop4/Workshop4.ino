/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 * @copyright Copyright (c) 2022
 */

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN_F = A4;
const byte SHARP_PIN_L = A5;
const byte SHARP_PIN_R = A3;

// Variables to store the proximity measurement
int sharp_val_F = 0;  // integer read from analog pin (F)
int sharp_val_L = 0;  // integer read from analog pin (L)
int sharp_val_R = 0;  // integer read from analog pin (R)
float sharp_range_F;  // range measurement [cm] (F)
float sharp_range_L;  // range measurement [cm] (L)
float sharp_range_R;  // range measurement [cm] (R)

void setup() {
  // Open the serial port at 115200 bps
  Serial.begin(115200);
}

void loop() {
  // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
  sharp_val_F= analogRead(SHARP_PIN_F);
  sharp_val_L= analogRead(SHARP_PIN_L);
  sharp_val_R = analogRead(SHARP_PIN_R);
  
  sharp_range_F = sharp_distance(sharp_val_F);
  sharp_range_L = sharp_distance(sharp_val_L);
  sharp_range_R = sharp_distance(sharp_val_R);


  // Print all values
  Serial.print("Front: ");
  Serial.print(sharp_range_F);
  Serial.print(" cm | Left: ");
  Serial.print(sharp_range_L);
  Serial.print(" cm | Right: ");
  Serial.print(sharp_range_R);
  Serial.print(" cm");
  Serial.print("\n");

  // Delay for a bit before reading the sensor again
  delay(100);
}

float sharp_distance(int integer_val) {
  float distance;
  distance=85.48*exp(-0.004*integer_val);
  return distance;
}
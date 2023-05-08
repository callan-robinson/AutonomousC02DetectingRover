/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.0
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

// Motor driver PWM pin
const byte E1 = 5;
const byte E2 = 3;
// Motor driver direction pin
int I1 = 2; // Wheel direction digital pin 2, right side
int I2 = 4; // Wheel direction digital pin 4, right side
int I3 = 7; // Wheel direction digital pin 7, left side
int I4 = 8; // Wheel direction digital pin 8, left side
// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte L_SIGNAL_A = 12;
const byte L_SIGNAL_B = 13;

// Right wheel encoder digital pins
const byte R_SIGNAL_A = 10;
const byte R_SIGNAL_B = 11;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long L_encoder_ticks = 0;
volatile long R_encoder_ticks = 0;

// Variable to store estimated angular rate of left and rightwheel [rad/s]
double omega_L = 0.0;

double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void L_decodeEncoderTicks()
{
    if (digitalRead(L_SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        L_encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        L_encoder_ticks++;
    }
}

void R_decodeEncoderTicks()
{
    // Right wheel encoder ticks
    if (digitalRead(R_SIGNAL_B) == LOW)
    {
        R_encoder_ticks++;
    }
    else
    {
        R_encoder_ticks--;
    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(L_SIGNAL_A, INPUT);
    pinMode(L_SIGNAL_B, INPUT);

    pinMode(R_SIGNAL_A, INPUT);
    pinMode(R_SIGNAL_B, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(L_SIGNAL_A), L_decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(R_SIGNAL_A), R_decodeEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)L_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)R_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        Serial.print("Left Encoder ticks: ");
        Serial.print(L_encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print(" | ");

        Serial.print("Right Encoder ticks: ");
        Serial.print(R_encoder_ticks);
        Serial.print(" Estimated Right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        L_encoder_ticks = 0;
        R_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Write to the output pins
    digitalWrite(I3, LOW); // Drive forward (left wheels)
    digitalWrite(I4, HIGH);
    analogWrite(E1, u); // Write left motors command

    digitalWrite(I1, HIGH); // Drive forward (right wheels)
    digitalWrite(I2, LOW);
    analogWrite(E2, u); // Write right motors command
}
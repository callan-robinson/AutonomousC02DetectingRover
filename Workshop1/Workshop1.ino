int EA = 3; // Wheel PWM pin (must be a PWM pin)
int EB = 5; // Wheel PWM pin (must be a PWM pin)
int I1 = 2; // Wheel direction digital pin 2
int I2 = 4; // Wheel direction digital pin 4
int I3 = 7; // Wheel direction digital pin 7
int I4 = 8; // Wheel direction digital pin 8

void setup()
{
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);
}

void loop()
{
    int u = 150; // A variable for the motor PWM command [0-255]
    // Drive forward slowly
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(3000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Drive backwards slowly
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(3000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    u = 255;
    // Drive forward fast
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(3000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Drive backwards fast
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(3000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Clockwise 360
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(2430);

    // Pause
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Counterclockwise 360
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, u);
    delay(2430);

    // Pause
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Clockwise circle
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EA, 70);
    analogWrite(EB, u);
    delay(6000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Counterclockwise circle
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, 90);
    delay(5000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(3000);

    // Figure Eight
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EA, 70);
    analogWrite(EB, u);
    delay(3200);

    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    analogWrite(EA, u);
    delay(500);

    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, 100);
    delay(3600);

    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EB, u);
    delay(1000);

    // Stop
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(5000);
}
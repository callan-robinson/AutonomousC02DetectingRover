int EA = 3; // Wheel PWM pin (must be a PWM pin)
int EB = 5; // Wheel PWM pin (must be a PWM pin)
int I1 = 2; // Wheel direction digital pin 2
int I2 = 4; // Wheel direction digital pin 4
int I3 = 7; // Wheel direction digital pin 7
int I4 = 8; // Wheel direction digital pin 8

const byte L_SIGNAL_A = 12; // Left wheel encoder digital pins
const byte L_SIGNAL_B = 13;
const byte R_SIGNAL_A = 10; // Right wheel encoder digital pins
const byte R_SIGNAL_B = 11;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// robot body width
const double ELL = 0.2775;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long L_encoder_ticks = 0;
volatile long R_encoder_ticks = 0;

// Current velocity and steering rate
typedef struct State
{
    double v_left, v_right;
} state;

state current;

// Set desired velocity and steering rate
double desired_velocity = 0.7;
double desired_steering_rate = 0;

// Creating variables for navigation and controll
int kp = 250, ki = 350;
double omega_L, omega_R, v_L, v_R, right_desired, left_desired, error, proportional, integral_L = 0, integral_R = 0;
short u_L = 0, u_R = 0;

// Create variables to track time intervals
unsigned long tick_clock;
unsigned long controller_clock;

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

// compute vehicle speed [m/s]
void compute_vehicle_state()
{
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)L_encoder_ticks / (double)TPR) * 1000.0 / (double)((millis()) - tick_clock);
    omega_R = 2.0 * PI * ((double)R_encoder_ticks / (double)TPR) * 1000.0 / (double)((millis()) - tick_clock);

    // Reset the encoder ticks counter and tick_clock
    L_encoder_ticks = 0;
    R_encoder_ticks = 0;
    tick_clock = millis();

    // estimate translational velocity [m/s]
    v_L = RHO * omega_L;
    v_R = RHO * omega_R;

    current.v_left = v_L;
    current.v_right = v_R;
}

short right_pi_controller(double estimated_vr, double vd, double wd)
{
    right_desired = vd + (wd * ELL / 2);
    error = right_desired - estimated_vr;
    proportional = kp * error;
    integral_R += ki * error * ((millis() - controller_clock) / 1000.0);

    if (isnan(integral_R))
    {
        integral_R = 0;
    }

    u_R = (short)(proportional + integral_R);

    Serial.print("\t\tRight Speed: ");
    Serial.print(estimated_vr);

    Serial.print("  Right Error: ");
    Serial.print(error);

    if (u_R < 0)
    {
        u_R = abs(u_R);
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    }
    else
    {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    }
    if (u_R > 255)
    {
        integral_R -= 20;
        u_R = 255;
    }

    Serial.print("   uR: ");
    Serial.println(u_R);

    return u_R;
};

short left_pi_controller(double estimated_vl, double vd, double wd)
{
    left_desired = vd - (wd * ELL / 2);
    error = left_desired - estimated_vl;
    proportional = kp * error;
    integral_L += ki * error * ((millis() - controller_clock) / 1000.0);

    if (isnan(integral_L))
    {
        integral_L = 0;
    }

    u_L = (short)(proportional + integral_L);

    Serial.print("Left Speed: ");
    Serial.print(estimated_vl);

    Serial.print("  Left Error: ");
    Serial.print(error);

    if (u_L < 0)
    {
        u_L = abs(u_L);
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    else
    {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    }
    if (u_L > 255)
    {
        integral_L -= 20;
        u_L = 255;
    }

    Serial.print("    uL: ");
    Serial.print(u_L);

    return u_L;
};

void setup()
{
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(L_SIGNAL_A, INPUT);
    pinMode(L_SIGNAL_B, INPUT);
    pinMode(R_SIGNAL_A, INPUT);
    pinMode(R_SIGNAL_B, INPUT);

    // Start counting time just before tick counting begins
    tick_clock = millis();

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(L_SIGNAL_A), L_decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(R_SIGNAL_A), R_decodeEncoderTicks, RISING);

    controller_clock = millis();

    Serial.begin(9600);
}

void loop()
{
    compute_vehicle_state();

    analogWrite(EB, left_pi_controller(current.v_left, desired_velocity, desired_steering_rate));
    analogWrite(EA, right_pi_controller(current.v_right, desired_velocity, desired_steering_rate));
    controller_clock = millis();

    delay(100);
}
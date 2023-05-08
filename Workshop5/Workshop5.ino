/**
 * @file c02-measurement-sgp30.ino
 * @author Thomas Sears (thomas.sears@queensu.ca)
 * @brief Basic functions of reading eCO2 measurements from the SGP30 sensor
 * @version 1.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

// Need this library installed to read from the SGP30
// For more details, look here: https://github.com/adafruit/Adafruit_SGP30
// and https://adafruit.github.io/Adafruit_SGP30/html/index.html
#include <Adafruit_SGP30.h>

Adafruit_SGP30 sgp;

/* return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity);

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);

    // Wait for serial connection before starting
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("__SGP30 demo__");

    if (!sgp.begin())
    {
        Serial.println("Sensor not found :(");
        while (1)
        {
            delay(10); // This will stay here forever if a sensor isn't found
        }
    }

    // Each board has a unique serial number that you can read
    // if you want to track which sensor you are using
    Serial.print("SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
}

int counter = 0;
unsigned long period = 1000; // Measurement period [ms]
unsigned long measurement_time = millis();

void loop()
{
    if (millis() - measurement_time > period)
    {
        measurement_time = millis();

        if (!sgp.IAQmeasure())
        {
            Serial.println("Measurement failed");
            return;
        }
        Serial.print("TVOC ");
        Serial.print(sgp.TVOC);
        Serial.print(" ppb\t");
        Serial.print("eCO2 ");
        Serial.print(sgp.eCO2);
        Serial.println(" ppm");
    }
}

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]

    return absoluteHumidityScaled;
}

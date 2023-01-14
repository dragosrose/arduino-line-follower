#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

// increase kp’s value and see what happens
float kp = 20;
float ki = 0.1;
float kd = 15;

int p = 0;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;
unsigned long previousTime = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 150;

unsigned long last = millis();
const int INTERVAL_MS = 1000;

enum Direction
{
    LEFT,
    RIGHT
};

Direction dir = LEFT;

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
    // pinMode setup
    pinMode(m11Pin, OUTPUT);
    pinMode(m12Pin, OUTPUT);
    pinMode(m21Pin, OUTPUT);
    pinMode(m22Pin, OUTPUT);
    pinMode(m1Enable, OUTPUT);
    pinMode(m2Enable, OUTPUT);

    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
    qtr.setEmitterPin(2);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    calibrate();
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);
    // for (uint8_t i = 0; i < SensorCount; i++)
    // {
    //     Serial.print(qtr.calibrationOn.minimum[i]);
    //     Serial.print(' ');
    // }
    // Serial.println();

    // // print the calibration maximum values measured when emitters were on
    // for (uint8_t i = 0; i < SensorCount; i++)
    // {
    //     Serial.print(qtr.calibrationOn.maximum[i]);
    //     Serial.print(' ');
    // }
    // Serial.println();
    // Serial.println();
    delay(1000);
}

void loop()
{
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    // for (uint8_t i = 0; i < SensorCount; i++)
    // {
    //     Serial.print(sensorValues[i]);
    //     Serial.print('\t');
    // }
    // Serial.println(position);

    int error = map(position, 0, 5000, -10, 10);

    PID(error);

    // delay(250);
}

void PID(int error)
{
    p = error;
    i = i + error;
    d = error - lastError;
    lastError = error;

    int motorspeed = kp * p + ki * i + kd * d;
    Serial.println(motorspeed);

    m1Speed = baseSpeed;
    m2Speed = baseSpeed;

    if (error < 0)
    {
        m1Speed += motorspeed;
    }
    else if (error > 0)
    {
        m2Speed -= motorspeed;
    }

    m1Speed = constrain(m1Speed, -255, maxSpeed);
    m2Speed = constrain(m2Speed, -255, maxSpeed);

    setMotorSpeed(m1Speed, m2Speed);
    Serial.print(m1Speed);
    Serial.print("\t");
    Serial.println(m2Speed);
}

void setMotorSpeed(int motor1Speed, int motor2Speed)
{
    // remove comment if any of the motors are going in reverse
    //  motor1Speed = -motor1Speed;
    //  motor2Speed = -motor2Speed;
    if (motor1Speed == 0)
    {
        digitalWrite(m11Pin, LOW);
        digitalWrite(m12Pin, LOW);
        analogWrite(m1Enable, motor1Speed);
    }
    else
    {
        if (motor1Speed > 0)
        {
            digitalWrite(m11Pin, HIGH);
            digitalWrite(m12Pin, LOW);
            analogWrite(m1Enable, motor1Speed);
        }
        if (motor1Speed < 0)
        {
            digitalWrite(m11Pin, LOW);
            digitalWrite(m12Pin, HIGH);
            analogWrite(m1Enable, -motor1Speed);
        }
    }
    if (motor2Speed == 0)
    {
        digitalWrite(m21Pin, LOW);
        digitalWrite(m22Pin, LOW);
        analogWrite(m2Enable, motor2Speed);
    }
    else
    {
        if (motor2Speed > 0)
        {
            digitalWrite(m21Pin, HIGH);
            digitalWrite(m22Pin, LOW);
            analogWrite(m2Enable, motor2Speed);
        }
        if (motor2Speed < 0)
        {
            digitalWrite(m21Pin, LOW);
            digitalWrite(m22Pin, HIGH);
            analogWrite(m2Enable, -motor2Speed);
        }
    }
}

void calibrate()
{
    int negVal = -150;
    int posVal = 120;

    if (dir == LEFT)
    {
        setMotorSpeed(0, posVal);
    }
    else
    {
        setMotorSpeed(0, negVal);
    }

    int crtWindowIdx = 1;
    while (millis() < 10000)
    {
        if (millis() > crtWindowIdx * 500)
        {
            dir = dir == LEFT ? RIGHT : LEFT;
            crtWindowIdx++;

            if (dir == LEFT)
            {
                setMotorSpeed(0, posVal);
            }
            else
            {
                setMotorSpeed(0, negVal);
            }
        }

        qtr.calibrate();
    }

    setMotorSpeed(0, 0);
}

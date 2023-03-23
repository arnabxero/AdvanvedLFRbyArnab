#include <QTRSensors.h>
#include "TB67H420FTG.h"
///////https://www.youtube.com/watch?v=PP4fvBVe3rI///////

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {A9, A8, A7, A6, A5, A4, A3, A2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Properties
TB67H420FTG driver(6, 5, 9, 8, 7, 10);

// PID Properties
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 3500;
const unsigned char MAX_SPEED = 50;


void setup() {
  driver.init();

  // Initialize line sensor array
  calibrateLineSensor();
}

void loop() {

  // Get line position
  unsigned int position = qtra.readLine(sensorValues);

  // Compute error from line
  int error = GOAL - position;

  // Compute motor adjustment
  int adjustment = KP*error + KD*(error - lastError);

  // Store error for next increment
  lastError = error;

  // Adjust motors 
  driver.setMotorAPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));

}

void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 3000; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}

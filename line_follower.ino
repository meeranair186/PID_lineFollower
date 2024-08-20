const int motorAPin1 = 8;
const int motorAPin2 = 9;
const int motorBPin1 = 12;
const int motorBPin2 = 13;
const int enableAPin = 10;
const int enableBPin = 11;

// IR Sensor pins
const int numSensors = 7; // Number of IR sensors
const int sensorPins[] = {2, 3, 4, 5, 6, 7, A0};

// PID params (adjust)
const float kp = 0.7;  // Proportional
const float ki = 0.001; // Integral
const float kd = 0; // Derivative
const int target = numSensors / 2; // Target sensor index (middle sensor)

// Variables for PI
float integral = 0.0;
float derivative = 0;
float lastError = 0.0;

unsigned long time;
unsigned long prev_time = -1;
int error;
int prev_error = 0;

// Constant out
const int enable = 100;

// Scaling
const int scale = 15;

void setup() {
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);

  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  analogWrite(enableAPin, enable);
  analogWrite(enableBPin, enable);
}

void loop() {

  time = millis();
  Serial.println(time);

  int sensorValues[numSensors];

  // Read sensor values
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }

  // Motor driver input
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorBPin1, HIGH);

  // Calculate error
  error = calculateError(sensorValues);

  // Reset integral term if error is close to zero
  if (abs(error) < 10) {
    integral = 0;
  }

  // Calculate control output using PI algorithm
  float controlOutput = kp * error + ki * integral + kd * derivative;
  Serial.println(controlOutput);

  // Apply control output to motor speeds
  int leftSpeed = constrain(enable - controlOutput, 0, 255);
  int rightSpeed = constrain(enable + controlOutput + 20, 0, 255);
  Serial.print("Left speed :");
  Serial.println(enable - controlOutput);
  Serial.print("Right speed :");
  Serial.println(enable + controlOutput);

  // Apply speed to motors
  analogWrite(enableAPin, leftSpeed);
  analogWrite(enableBPin, rightSpeed);

  // Update integral term
  integral = constrain(integral + error, -1000, 1000); // Adjust limits as needed
  derivative = (error - prev_error) / (time - prev_time);

  prev_time = time;
  prev_error = error;
}

int calculateError(int sensorValues[]) {
  int error = 0;
  for (int i = 0; i < numSensors; i++) {
    error += (i - target) * sensorValues[i];
  }
  return error * scale;
}

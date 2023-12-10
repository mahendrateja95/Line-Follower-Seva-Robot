#include <Ultrasonic.h>

#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enA 10
#define enB 5
#define triggerPin 12
#define echoPin 11
#define buzzerPin 13

Ultrasonic ultrasonic(triggerPin, echoPin);
bool objectDetected = false;
unsigned long stopTime = 0;
unsigned long resumeTime = 0;
unsigned long ultrasonicTimeout = 180000; // 3 minutes in milliseconds
unsigned long stopDuration = 30000; // 2 minutes in milliseconds

int M1_Speed = 100; // speed of motor 1
int M2_Speed = 100; // speed of motor 2
int LeftRotationSpeed = 250; // Left Rotation Speed
int RightRotationSpeed = 250; // Right Rotation Speed

unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 2000; // 2 seconds in milliseconds
unsigned long buzzerOffTime = 2000; // 2 seconds in milliseconds

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Initialize the ultrasonic sensor
  ultrasonic.setTimeout(50000); // Set a timeout in microseconds

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if the ultrasonic sensor should be active
  if (millis() < resumeTime) {
    delay(100); // Small delay to reduce the loop frequency when the sensor is inactive
    return;
  }

  // Check for object detection using the ultrasonic sensor
  long distance = ultrasonic.read();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 40) {
    Stop();
    objectDetected = true;
    stopTime = millis();
    resumeTime = stopTime + stopDuration;
    buzzerStartTime = millis(); // Start the buzzer timer
  } else {
    objectDetected = false;
  }

  if (objectDetected) {
    // Check the buzzer timer
    unsigned long elapsedBuzzerTime = millis() - buzzerStartTime;

    if (elapsedBuzzerTime < buzzerDuration) {
      digitalWrite(buzzerPin, HIGH);
    } else if (elapsedBuzzerTime < (buzzerDuration + buzzerOffTime)) {
      digitalWrite(buzzerPin, LOW);
    } else {
      // Reset the buzzer timer for the next cycle
      buzzerStartTime = millis();
    }
  } else {
    // React based on other sensor inputs when no object is detected
    int LEFT_SENSOR = digitalRead(A0);
    int RIGHT_SENSOR = digitalRead(A1);

    if (RIGHT_SENSOR == LOW && LEFT_SENSOR == LOW) {
      forward(); // FORWARD
    } else if (RIGHT_SENSOR == LOW && LEFT_SENSOR == HIGH) {
      right(); // Move Right
    } else if (RIGHT_SENSOR == HIGH && LEFT_SENSOR == LOW) {
      left(); // Move Left
    } else if (RIGHT_SENSOR == HIGH && LEFT_SENSOR == HIGH) {
      Stop(); // STOP
    }
  }
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void left() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(buzzerPin, LOW); // Turn off the buzzer
}

#include <Servo.h>
#include <NewPing.h>

// Motor control pins for Motor 1
#define MOTOR1_ENABLE_PIN 9
#define MOTOR1_PIN1 22
#define MOTOR1_PIN2 23

// Motor control pins for Motor 2
#define MOTOR2_ENABLE_PIN 10
#define MOTOR2_PIN1 24
#define MOTOR2_PIN2 25

// Ultrasonic sensor pins and max distance
#define TRIG_PIN 26
#define ECHO_PIN 27
#define MAX_DISTANCE 400  // Maximum distance to measure (in cm)

// Servo motor pin
#define SERVO_PIN 11

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Motor Control Class
class MotorController {
public:
    void motorControl(int enablePin, int pin1, int pin2, int speed, int dir1, int dir2) {
        analogWrite(enablePin, speed);
        digitalWrite(pin1, dir1);
        digitalWrite(pin2, dir2);
    }
    
    void moveForward() {
        Serial.println("Moving forward...");
        motorControl(MOTOR1_ENABLE_PIN, MOTOR1_PIN1, MOTOR1_PIN2, 165, HIGH, LOW);
        motorControl(MOTOR2_ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, 165, HIGH, LOW);
    }
    
    void stop() {
        motorControl(MOTOR1_ENABLE_PIN, MOTOR1_PIN1, MOTOR1_PIN2, 0, LOW, LOW);
        motorControl(MOTOR2_ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, 0, LOW, LOW);
    }
    
    void turnLeft() {
        Serial.println("Turning Left...");
        motorControl(MOTOR1_ENABLE_PIN, MOTOR1_PIN1, MOTOR1_PIN2, 150, HIGH, LOW);
        motorControl(MOTOR2_ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, 150, LOW, HIGH);

        // delay(1200);
        
        unsigned long startTime = millis();
        while (millis() - startTime < 1100) {
            // Keep running motors without blocking other processes
        }

        stop();
    }
    
    void turnRight() {
        Serial.println("Turning Right...");
        motorControl(MOTOR1_ENABLE_PIN, MOTOR1_PIN1, MOTOR1_PIN2, 150, LOW, HIGH);
        motorControl(MOTOR2_ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, 150, HIGH, LOW);

        unsigned long startTime = millis();
        while (millis() - startTime < 1100) {
            // Keep running motors without blocking other processes
        }

        // delay(1200);
        stop();
    }
    
    void turnAround() {
        Serial.println("Turning Around...");
        motorControl(MOTOR1_ENABLE_PIN, MOTOR1_PIN1, MOTOR1_PIN2, 150, HIGH, LOW);
        motorControl(MOTOR2_ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, 150, LOW, HIGH);

        unsigned long startTime = millis();
        while (millis() - startTime < 1100) {
            // Keep running motors without blocking other processes
        }

        // delay(2200);
        stop();
    }
};

// Sensor Handling Class
// Improved Sensor Handling Class
class SensorHandler {
private:
    int trigPin, echoPin;
    Servo headServo;
    int servoPin;
    int currentAngle;  // Track last position

public:
    SensorHandler(int trig, int echo, int servoPin) 
        : trigPin(trig), echoPin(echo), servoPin(servoPin), currentAngle(90) {}

    void init() {
        headServo.attach(servoPin);
        headTurn(90); // Ensure it starts facing forward
        delay(1500);
    }

    long getDistance() {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        return pulseIn(echoPin, HIGH) * 0.034 / 2; // Convert time to distance
    }

    void headTurn(int angle) {
        if (angle != currentAngle) {  // Move only if necessary
            headServo.write(angle);
            delay(abs(currentAngle - angle) * 2); // Adaptive delay
            currentAngle = angle;
        }
    }

    long assignDistance(int angle) {
        headTurn(angle);
        delay(200); // Give some time to stabilize reading
        return getDistance();
    }
};

// AI Decision Class
class AIController {
private:
    MotorController motor;
    SensorHandler& sensor;
public:
    AIController(SensorHandler& s) : sensor(s) {}

    void makeDecision() {
        motor.moveForward();
        long front = sensor.getDistance();

        Serial.println("front: " + String(front));

        if (front < 50) {
            motor.stop();

            long right = sensor.assignDistance(-90);
            Serial.println("right: " + String(right));
            delay(200);
            long left = sensor.assignDistance(180);
            Serial.println("left: " + String(left));
            delay(200);
            sensor.headTurn(90); // Reset to front
            delay(200);
            
            front = sensor.getDistance(); // Check front again
            Serial.println("front (error): " + String(front));

            if (front < 80) {
                if (left < 50 && right < 50) {
                    motor.turnAround();
                } else if (left > 50 && right > 50) {
                    motor.turnLeft();
                } else if (left > 50) {
                    motor.turnLeft();
                } else if (right > 50) {
                    motor.turnRight();
                }
            }
        }
    }
};

// Global Objects (Now includes Servo pin)
SensorHandler sensor(TRIG_PIN, ECHO_PIN, SERVO_PIN);
AIController ai(sensor);

void setup() {
    pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);

    pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.begin(9600);

    sensor.init();  // Attach servo here
}

void loop() {
    ai.makeDecision();
    delay(500);
}
/**
 * ============================================================================
 * Ball and Beam PID Controller
 * ============================================================================
 * 
 * A PID-based control system for balancing a car on a beam using Arduino.
 * 
 * @project   Object Balancing on a Beam using PID Controller
 * @course    EEE 318 - Control Systems Laboratory (January 2024)
 * @section   A1, Group 03
 * @institution Bangladesh University of Engineering and Technology (BUET)
 * 
 * @authors
 *   - Sayba Kamal Orni (2006009)
 *   - Soumik Saha (2006011)
 *   - Adith Saha Dipro (2006018)
 *   - Md. Adib Rahman (2006024)
 *   - Nittya Ananda Biswas (2006025)
 *   - Khondakar Ashik Shahriar (2006026)
 * 
 * @description
 *   This system uses a PID controller to maintain a car's position on a beam
 *   by dynamically adjusting the beam's tilt angle via a servo motor. Position
 *   feedback is provided by ultrasonic (sonar) sensors.
 * 
 * @hardware
 *   - Arduino Uno
 *   - 2x HC-SR04 Ultrasonic Sensors
 *   - 1x MG996R Servo Motor
 *   - Car with reflective surface
 *   - Beam assembly
 * 
 * @version 1.0
 * @date December 2024
 * 
 * ============================================================================
 */

#include <Servo.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// Control Output Limits
#define UMAX_DEG        30          // Maximum control angle (degrees)
#define UMIN_DEG       -30          // Minimum control angle (degrees)
#define UMAX_RAD        0.524       // Maximum control angle (radians)
#define UMIN_RAD       -0.524       // Minimum control angle (radians)

// Pin Definitions
const int ECHO_PIN_SETPOINT = 5;    // Sensor 1 echo (setpoint measurement)
const int TRIG_PIN_SETPOINT = 6;    // Sensor 1 trigger
const int ECHO_PIN_POSITION = 11;   // Sensor 2 echo (car position)
const int TRIG_PIN_POSITION = 10;   // Sensor 2 trigger
const int SERVO_PIN = 9;            // Servo control pin

// PID Controller Parameters (Tuned using Ziegler-Nichols Method)
const double KP = 8.4;              // Proportional gain
const double KI = 3.73;             // Integral gain
const double KD = 8.0;              // Derivative gain

// Servo Angle Mapping
const int SERVO_NEUTRAL = 90;       // Neutral position (degrees)
const int SERVO_MAX = 130;          // Maximum servo angle
const int SERVO_MIN = 60;           // Minimum servo angle

// Error Thresholds for Adaptive Control
const double ERROR_SMALL = 0.0035;  // Small error threshold (m)
const double ERROR_LARGE = 0.03;    // Large error threshold (m)
const int STEP_SMALL = 3;           // Small adjustment step (degrees)
const int STEP_LARGE = 5;           // Large adjustment step (degrees)

// Stuck Detection Parameters
const double ERROR_STUCK_THRESHOLD = 0.1;   // 10% of setpoint
const unsigned long STUCK_TIME_MS = 3000;   // 3 seconds
const int STUCK_ANGLE_LOW = 40;             // Recovery angle (low)
const int STUCK_ANGLE_HIGH = 140;           // Recovery angle (high)

// Sensor Configuration
const int NUM_SAMPLES = 50;                 // Samples for averaging
const unsigned long ECHO_TIMEOUT_US = 60000; // Echo timeout

// Filter Coefficients
const double SETPOINT_FILTER_COEFF = 0.5;   // Setpoint filter
const double POSITION_FILTER_COEFF = 0.53;  // Position filter
const double DERIVATIVE_FILTER_COEFF = 0.56; // Derivative filter

// Loop Timing
const int LOOP_DELAY_MS = 30;               // Main loop delay

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Servo servo;

// Control Variables
double setpoint = 0.0;
double setpoint_prev = 0.0;
double car_position = 0.0;
double car_position_prev = 0.0;
double error = 0.0;

// PID Terms
double proportional = 0.0;
double integral = 0.0;
double derivative = 0.0;
double control_output = 0.0;

// Previous Values
double integral_prev = 0.0;
double derivative_prev = 0.0;

// Timing
double dt = 0.0;
double last_time = 0.0;

// Servo Control
int target_angle = SERVO_NEUTRAL;
int current_angle = SERVO_NEUTRAL;

// System Flags
boolean is_saturated = false;
boolean is_stuck = false;
unsigned long stuck_start_time = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

float measureSetpoint();
float measureCarPosition();
void computePID();
void applySaturation();
void updateServoAngle();
void detectStuckCondition(unsigned long currentTime);
void handleStuckCondition();
void moveServo(int angle);

// ============================================================================
// SETUP - Initialize System
// ============================================================================

void setup() {
    // Initialize Serial Communication
    Serial.begin(9600);
    
    // Configure Sensor Pins
    pinMode(TRIG_PIN_SETPOINT, OUTPUT);
    pinMode(ECHO_PIN_SETPOINT, INPUT);
    pinMode(TRIG_PIN_POSITION, OUTPUT);
    pinMode(ECHO_PIN_POSITION, INPUT);
    
    // Initialize Servo
    servo.attach(SERVO_PIN);
    moveServo(SERVO_NEUTRAL);
    
    // Initialize Timing
    last_time = 0;
    
    // System Warm-up
    delay(1000);
    
    // Initialize Setpoint and Position
    setpoint_prev = measureSetpoint();
    delay(2);
    car_position_prev = measureCarPosition();
    delay(1);
    
    // Print CSV Header
    Serial.println("Timestamp(ms),Setpoint(cm),Position(cm)");
}

// ============================================================================
// MAIN LOOP - Control System
// ============================================================================

void loop() {
    // Calculate Time Step
    unsigned long now = millis();
    dt = (now - last_time) / 1000.0;
    last_time = now;
    
    // === SENSOR READING & FILTERING ===
    
    // Read and Filter Setpoint
    setpoint = measureSetpoint();
    setpoint = SETPOINT_FILTER_COEFF * setpoint + 
               (1.0 - SETPOINT_FILTER_COEFF) * setpoint_prev;
    
    // Read and Filter Car Position
    car_position = measureCarPosition();
    car_position = POSITION_FILTER_COEFF * car_position + 
                   (1.0 - POSITION_FILTER_COEFF) * car_position_prev;
    
    // Calculate Position Error
    error = car_position - setpoint;
    
    // === STUCK DETECTION & RECOVERY ===
    
    detectStuckCondition(now);
    
    if (is_stuck) {
        handleStuckCondition();
        // Skip PID control while recovering from stuck condition
        return;
    }
    
    // === PID CONTROL ===
    
    computePID();
    applySaturation();
    
    // === SERVO CONTROL ===
    
    // Convert Control Output to Servo Angle
    control_output = round(control_output * (180.0 / M_PI));
    target_angle = map(control_output, UMAX_DEG, UMIN_DEG, SERVO_MAX, SERVO_MIN);
    
    // Apply Adaptive Servo Movement
    updateServoAngle();
    
    // === DATA LOGGING ===
    
    Serial.print(now);
    Serial.print(",");
    Serial.print(setpoint * 100.0, 3);  // Convert to cm
    Serial.print(",");
    Serial.println(car_position * 100.0, 3);  // Convert to cm
    
    // === UPDATE PREVIOUS VALUES ===
    
    integral_prev = integral;
    car_position_prev = car_position;
    derivative_prev = derivative;
    setpoint_prev = setpoint;
    
    // Control Loop Timing
    delay(LOOP_DELAY_MS);
}

// ============================================================================
// PID CONTROL IMPLEMENTATION
// ============================================================================

/**
 * Compute PID control output based on current error
 */
void computePID() {
    // Proportional Term
    proportional = KP * error;
    
    // Integral Term (with anti-windup)
    if (!is_saturated) {
        integral = integral_prev + dt * KI * error;
    }
    
    // Derivative Term (with filtering)
    derivative = (KD / dt) * (car_position - car_position_prev);
    derivative = DERIVATIVE_FILTER_COEFF * derivative + 
                 (1.0 - DERIVATIVE_FILTER_COEFF) * derivative_prev;
    
    // Total Control Output
    control_output = proportional + integral + round(100.0 * derivative) * 0.01;
}

/**
 * Apply saturation limits to prevent actuator overrun
 */
void applySaturation() {
    if (control_output < UMIN_RAD) {
        control_output = UMIN_RAD;
        is_saturated = true;
    } else if (control_output > UMAX_RAD) {
        control_output = UMAX_RAD;
        is_saturated = true;
    } else {
        is_saturated = false;
    }
}

// ============================================================================
// SERVO CONTROL WITH ADAPTIVE SPEED
// ============================================================================

/**
 * Update servo angle with variable speed based on error magnitude
 * Larger errors result in faster servo movement
 */
void updateServoAngle() {
    int step_size = 0;
    
    // Determine Step Size Based on Error
    if (abs(error) >= ERROR_SMALL && abs(error) < ERROR_LARGE) {
        step_size = STEP_SMALL;  // Gentle correction for small errors
    } else if (abs(error) >= ERROR_LARGE) {
        step_size = STEP_LARGE;  // Aggressive correction for large errors
    } else {
        return;  // Error too small, no movement needed
    }
    
    // Apply Step to Current Angle
    if (current_angle < target_angle) {
        current_angle = min(current_angle + step_size, target_angle);
    } else if (current_angle > target_angle) {
        current_angle = max(current_angle - step_size, target_angle);
    }
    
    // Command Servo
    servo.write(current_angle);
}

// ============================================================================
// STUCK DETECTION & RECOVERY
// ============================================================================

/**
 * Detect if the car is stuck outside acceptable range
 */
void detectStuckCondition(unsigned long currentTime) {
    // Check if error exceeds threshold
    if (abs(error) > ERROR_STUCK_THRESHOLD * setpoint) {
        // Start timing if not already started
        if (stuck_start_time == 0) {
            stuck_start_time = currentTime;
        }
        
        // Mark as stuck if timeout exceeded
        if (currentTime - stuck_start_time > STUCK_TIME_MS) {
            is_stuck = true;
        }
    } else {
        // Reset stuck detection
        stuck_start_time = 0;
        is_stuck = false;
    }
}

/**
 * Handle stuck condition with aggressive servo movement
 */
void handleStuckCondition() {
    if (current_angle < SERVO_NEUTRAL) {
        // Car stuck on one side - tilt beam aggressively
        current_angle = STUCK_ANGLE_LOW;
    } else if (current_angle > SERVO_NEUTRAL) {
        // Car stuck on other side - tilt beam aggressively
        current_angle = STUCK_ANGLE_HIGH;
    }
    
    servo.write(current_angle);
}

// ============================================================================
// SENSOR MEASUREMENT FUNCTIONS
// ============================================================================

/**
 * Measure car position using Sensor 2 with 50-sample averaging
 * @return Average distance in meters
 */
float measureCarPosition() {
    float samples[NUM_SAMPLES];
    float sum = 0.0;
    
    // Collect Samples
    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Trigger Sensor
        digitalWrite(TRIG_PIN_POSITION, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN_POSITION, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN_POSITION, LOW);
        
        // Measure Echo Time
        long elapsed_time = pulseIn(ECHO_PIN_POSITION, HIGH, ECHO_TIMEOUT_US);
        
        // Convert to Distance
        // Formula: distance = (time * speed_of_sound) / 2
        // Speed of sound ≈ 343 m/s → 58.2 μs/cm
        float distance_cm = (float)elapsed_time / 58.2;
        samples[i] = distance_cm * 0.01;  // Convert to meters
        
        delay(1);
    }
    
    // Calculate Average
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += samples[i];
    }
    
    return sum / NUM_SAMPLES;
}

/**
 * Measure setpoint using Sensor 1 (single reading)
 * @return Distance in meters
 */
float measureSetpoint() {
    // Trigger Sensor
    digitalWrite(TRIG_PIN_SETPOINT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_SETPOINT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_SETPOINT, LOW);
    
    // Measure Echo Time
    long elapsed_time = pulseIn(ECHO_PIN_SETPOINT, HIGH, ECHO_TIMEOUT_US);
    
    // Convert to Distance (meters)
    float distance_cm = (float)elapsed_time / 58.2;
    return distance_cm * 0.01;
}

/**
 * Move servo to specified angle
 * @param angle Desired servo angle (degrees)
 */
void moveServo(int angle) {
    servo.write(angle);
}

// ============================================================================
// END OF CODE
// ============================================================================

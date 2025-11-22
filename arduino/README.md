# Arduino Code - Ball and Beam Controller

This directory contains the Arduino sketch for the Ball and Beam PID control system.

## Quick Start

1. **Hardware Setup**: Follow the main README for wiring diagram
2. **Open Code**: Load `ball_beam_controller.ino` in Arduino IDE
3. **Configure Board**: Select `Arduino Uno` (or your board)
4. **Upload**: Click Upload button (or Ctrl+U)
5. **Monitor**: Open Serial Monitor at 9600 baud

## Code Structure

```
ball_beam_controller.ino
├── Configuration Constants (Lines 19-72)
├── Global Variables (Lines 74-99)
├── Function Declarations (Lines 101-110)
├── setup() - Initialization (Lines 112-142)
├── loop() - Main Control Loop (Lines 144-207)
├── PID Functions (Lines 209-244)
├── Servo Control (Lines 246-284)
├── Stuck Detection (Lines 286-321)
└── Sensor Measurement (Lines 323-399)
```

## Key Parameters

### PID Gains (Lines 42-44)

```cpp
const double KP = 8.4;    // Proportional gain
const double KI = 3.73;   // Integral gain
const double KD = 8.0;    // Derivative gain
```

### Servo Mapping (Lines 47-49)

```cpp
const int SERVO_NEUTRAL = 90;   // Center position
const int SERVO_MAX = 130;      // Maximum angle
const int SERVO_MIN = 60;       // Minimum angle
```

### Error Thresholds (Lines 52-55)

```cpp
const double ERROR_SMALL = 0.0035;  // 3.5mm
const double ERROR_LARGE = 0.03;    // 30mm  
const int STEP_SMALL = 3;           // degrees
const int STEP_LARGE = 5;           // degrees
```

## Pin Configuration

| Arduino Pin | Component | Type |
|-------------|-----------|------|
| 5 | Sensor 1 Echo | INPUT |
| 6 | Sensor 1 Trig | OUTPUT |
| 9 | Servo Signal | PWM OUTPUT |
| 10 | Sensor 2 Trig | OUTPUT |
| 11 | Sensor 2 Echo | INPUT |

## Serial Output Format

CSV format: `Timestamp(ms),Setpoint(cm),Position(cm)`

Example:
```
1234,14.000,14.152
1264,14.000,13.987
1294,14.000,14.021
```

## Modifying Parameters

### To Adjust PID Gains

Edit lines 42-44:
```cpp
const double KP = 10.0;   // Increase for faster response
const double KI = 4.0;    // Increase to eliminate steady-state error
const double KD = 6.0;    // Increase to reduce oscillations
```

### To Change Sampling Rate

Edit line 72:
```cpp
const int LOOP_DELAY_MS = 50;  // Slower (20Hz)
const int LOOP_DELAY_MS = 10;  // Faster (100Hz)
```

### To Modify Sensor Averaging

Edit line 68:
```cpp
const int NUM_SAMPLES = 100;  // More samples = smoother, slower
const int NUM_SAMPLES = 25;   // Fewer samples = faster, noisier
```

## Troubleshooting

### Compilation Errors

- **Error**: `'M_PI' was not declared`
  - **Fix**: Update Arduino IDE to 1.8.19+

- **Error**: `'Servo' does not name a type`
  - **Fix**: Install Servo library via Library Manager

### Runtime Issues

- **Servo doesn't move**
  - Check connections to pin 9
  - Verify external power supply
  - Check servo.attach(9) in setup()

- **Erratic sensor readings**
  - Increase NUM_SAMPLES
  - Check sensor wiring
  - Ensure reflective surface on car

## Advanced Features

### Stuck Detection

Automatically triggers if error >10% of setpoint for >3 seconds.
Adjustable at lines 58-61.

### Adaptive Speed Control

Servo moves faster (5°/step) for large errors, slower (3°/step) for small errors.
Thresholds at lines 52-55.

### Digital Filtering

Low-pass filters on:
- Setpoint (coeff = 0.5)
- Position (coeff = 0.53)  
- Derivative (coeff = 0.56)

Edit coefficients at lines 70-72.

## License

MIT License - See main LICENSE file

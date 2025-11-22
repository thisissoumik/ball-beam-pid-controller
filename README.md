# Ball and Beam PID Controller 🎯

[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
[![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)

A comprehensive PID-based control system for balancing a car on a beam using Arduino, ultrasonic sensors, and servo motor actuation. This project demonstrates classical control theory principles applied to a real-world mechatronic system.

<p align="center">
  <i>Developed as part of EEE 318 - Control Systems Laboratory at Bangladesh University of Engineering and Technology (BUET)</i>
</p>

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [PID Tuning](#pid-tuning)
- [Project Structure](#project-structure)
- [Performance Metrics](#performance-metrics)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Team](#team)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## 🎯 Overview

The Ball and Beam system is a classic control engineering problem where the objective is to maintain a car at a desired position on a beam by adjusting the beam's tilt angle. This implementation uses:

- **PID Control Algorithm** for precise position control
- **Ultrasonic Sensors** for non-contact distance measurement  
- **Digital Filtering** for noise reduction
- **Anti-windup** mechanism to prevent integral term saturation
- **Adaptive Servo Control** with variable speed based on error magnitude
- **Stuck Detection & Recovery** for robust operation

### Key Innovations

- **Car Instead of Ball**: Replaced traditional ball with a car featuring a flat reflective surface for improved sensor accuracy
- **Stuck Detection**: Automatic detection and recovery when the car fails to reach the setpoint
- **Adaptive Speed Control**: Servo movement speed adjusts based on error magnitude
- **Position Limiting**: Physical barriers prevent sensor dead-zone errors

## ✨ Features

- ⚙️ **PID Control**: Tunable Proportional-Integral-Derivative controller
- 📊 **Digital Filtering**: Low-pass filters on setpoint, position, and derivative
- 🎚️ **Adaptive Servo Control**: Variable speed servo movement (3° for small errors, 5° for large)
- 🔒 **Anti-windup Protection**: Prevents integral windup during saturation
- 📈 **Data Logging**: CSV format output for analysis and plotting
- 🎯 **50-Sample Averaging**: Robust distance measurement with noise rejection
- 🚨 **Stuck Detection**: 3-second timeout with automatic recovery
- 🛡️ **Safety Features**: Angle saturation limits (±30°) and physical position limiters

## 🏗️ System Architecture

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Setpoint   │─────▶│    Error    │─────▶│     PID     │
│   Sensor    │      │ Calculation │      │ Controller  │
└─────────────┘      └─────────────┘      └─────────────┘
                           ▲                      │
                           │                      ▼
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Position   │      │   Digital   │      │    Servo    │
│   Sensor    │─────▶│   Filters   │      │    Motor    │
└─────────────┘      └─────────────┘      └─────────────┘
                                                 │
                                                 ▼
                                          ┌─────────────┐
                                          │ Beam & Car  │
                                          └─────────────┘
```

### Control Loop Flow

1. **Sensor Reading**: Measure setpoint and car position using ultrasonic sensors
2. **Digital Filtering**: Apply low-pass filters to reduce noise
3. **Error Calculation**: Compute deviation from setpoint
4. **Stuck Detection**: Check if car is stuck for >3 seconds
5. **PID Computation**: Calculate control output (if not stuck)
6. **Saturation**: Limit control output to ±30° (±0.524 rad)
7. **Angle Mapping**: Convert to servo angle range (60°-130°)
8. **Adaptive Movement**: Adjust servo with error-based speed
9. **Data Logging**: Output timestamp, setpoint, and position
10. **Repeat**: Loop continues at ~33Hz (30ms delay)

## 🔧 Hardware Requirements

### Components List

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| Arduino Uno | 1 | ATmega328P, 16MHz | Main controller |
| HC-SR04 Ultrasonic Sensor | 2 | 2-400cm range | Position sensing |
| MG996R Servo Motor | 1 | 180°, 10kg·cm torque | Beam actuation |
| Car (Toy) | 1 | Flat reflective surface | Object to balance |
| Wooden Beam | 1 | 30cm × 3cm × 1cm | Platform |
| Wooden Structure | 1 | Custom dimensions | Support frame |
| Reflective Board | 1 | 5cm × 3cm | Attached to car |
| Position Limiter Bar | 1 | Small wooden bar | Prevents sensor dead-zone |
| Breadboard | 1 | Standard | Prototyping |
| Jump Wires | ~20 | Male-to-male, Male-to-female | Connections |
| Power Supply | 1 | 5V, 2A+ | Arduino & servo power |

### Estimated Cost

**Total: ~৳3,520 BDT (≈ $30 USD)**

## 💻 Software Requirements

- **Arduino IDE** (v1.8.19 or higher) or **Arduino CLI**
- **Servo Library** (included with Arduino IDE)
- **MATLAB** (R2020a or higher) - For simulation and analysis
- **Serial Plotter/Monitor** - For real-time visualization

### Optional Tools

- **Python 3.x** with `matplotlib`, `pandas` - For advanced data analysis
- **Processing** - For custom visualization

## 📥 Installation

### 1. Clone the Repository

```bash
git clone https://github.com/soumik-saha/ball-beam-pid-controller.git
cd ball-beam-pid-controller
```

### 2. Hardware Assembly

1. **Build the Frame**: Construct wooden support structure
2. **Mount Servo**: Attach servo to base with lever arm connected to beam
3. **Attach Beam**: Connect beam to lever arm (ensure horizontal when servo at 90°)
4. **Position Sensors**:
   - Sensor 1: Mount at base for setpoint measurement
   - Sensor 2: Mount above beam for car position measurement
5. **Add Reflective Board**: Attach to front of car for better sensor accuracy
6. **Install Position Limiter**: Add small bar near Sensor 2 to prevent dead-zone errors
7. **Wire Connections**:

```
Arduino Uno Connections:
├─ Pin 5  → Sensor 1 Echo
├─ Pin 6  → Sensor 1 Trig
├─ Pin 9  → Servo Signal (Orange/Yellow)
├─ Pin 10 → Sensor 2 Trig
├─ Pin 11 → Sensor 2 Echo
├─ 5V     → Sensors VCC, Servo VCC (Red)
└─ GND    → Sensors GND, Servo GND (Brown/Black)
```

**⚠️ Important**: Use external 5V power supply (2A+) for servo to avoid overloading Arduino

### 3. Upload Arduino Code

1. Open Arduino IDE
2. Navigate to `File → Open` and select `arduino/ball_beam_controller.ino`
3. Select board: `Tools → Board → Arduino Uno`
4. Select port: `Tools → Port → [Your Arduino Port]`
5. Click **Upload** (→) or press `Ctrl+U`
6. Wait for "Done uploading" message

### 4. Verify Operation

1. Open **Serial Monitor** (`Tools → Serial Monitor`)
2. Set baud rate to **9600**
3. Place car on beam
4. Observe CSV output: `Timestamp(ms),Setpoint(cm),Position(cm)`

## 🚀 Usage

### Basic Operation

1. **Power Up**: Connect Arduino to computer or external power
2. **Calibration**:
   - Ensure beam is level when servo is at 90°
   - Adjust lever arm connection if needed
3. **Set Reference**: Place object at desired position on base rail
4. **Place Car**: Put car on beam (ensure reflective board faces Sensor 2)
5. **Monitor**: Open Serial Plotter to visualize real-time performance

### Serial Data Visualization

#### Using Arduino Serial Plotter

```bash
Tools → Serial Plotter
```
- Set baud rate: **9600**
- View real-time setpoint vs. position graph

#### Using MATLAB

```matlab
% Read CSV data from Serial Monitor
data = readmatrix('serial_data.csv');
time = data(:,1) / 1000;  % Convert to seconds
setpoint = data(:,2);
position = data(:,3);

% Plot
figure;
plot(time, setpoint, 'r--', 'LineWidth', 2);
hold on;
plot(time, position, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (cm)');
legend('Setpoint', 'Car Position');
title('Ball and Beam System Response');
grid on;
```

#### Using Python

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('serial_data.csv', 
                 names=['Timestamp', 'Setpoint', 'Position'])
df['Time'] = df['Timestamp'] / 1000  # Convert to seconds

# Plot
plt.figure(figsize=(12, 6))
plt.plot(df['Time'], df['Setpoint'], 'r--', label='Setpoint', linewidth=2)
plt.plot(df['Time'], df['Position'], 'b-', label='Car Position', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Position (cm)')
plt.title('Ball and Beam System Response')
plt.legend()
plt.grid(True)
plt.show()
```

## 🎛️ PID Tuning

### Current Parameters

The system is pre-tuned using the **Ziegler-Nichols Method**:

```cpp
const double KP = 8.4;    // Proportional gain
const double KI = 3.73;   // Integral gain  
const double KD = 8.0;    // Derivative gain
```

### Tuning Procedure

#### Step 1: Find Ultimate Gain (Ku)

```cpp
// Set Ki = 0, Kd = 0
const double KI = 0.0;
const double KD = 0.0;

// Gradually increase Kp until sustained oscillations occur
const double KP = 12.0;  // Example: oscillations at Ku = 12
```

Observe Serial Plotter - look for sinusoidal oscillations.

#### Step 2: Measure Ultimate Period (Tu)

Measure time between peaks: **Tu ≈ 4.5 seconds** (from experimental data)

#### Step 3: Calculate PID Parameters

Using Ziegler-Nichols formulas:

```
Kp = 0.6 * Ku = 0.6 × 12 = 7.2
Ki = 2 * Kp / Tu = 2 × 7.2 / 4.5 = 3.2
Kd = Kp * Tu / 8 = 7.2 × 4.5 / 8 = 4.05
```

#### Step 4: Fine-Tune

Empirically adjusted values for better performance:
- **Kp**: 8.4 (increased from 7.2 for faster response)
- **Ki**: 3.73 (slight increase to eliminate steady-state error)
- **Kd**: 8.0 (doubled for better damping)

### Manual Tuning Tips

1. **Too Much Oscillation?** → Decrease Kp, increase Kd
2. **Slow Response?** → Increase Kp
3. **Steady-State Error?** → Increase Ki (watch for overshoot)
4. **Overshooting?** → Decrease Kp, increase Kd

## 📁 Project Structure

```
ball-beam-pid-controller/
├── arduino/
│   ├── ball_beam_controller.ino    # Main Arduino code
│   └── README.md                    # Arduino-specific documentation
├── matlab/
│   ├── system_analysis.m            # Complete system analysis
│   └── plot_experimental_data.m     # Data visualization script
├── docs/
│   └── Project_Report.pdf           # Full technical report (38 pages)
├── .gitignore                       # Git ignore rules
├── LICENSE                          # MIT License
├── README.md                        # This file
├── CONTRIBUTING.md                  # Contribution guidelines
├── QUICKSTART.md                    # Quick start guide
└── CHANGELOG.md                     # Version history
```

## 📊 Performance Metrics

### Test Results

#### Test Case 1: Setpoint = 12.44 cm

- **Steady-State Error**: 0.153 cm (1.23%)
- **Percentage Overshoot**: 75.71%
- **Peak Time**: 0.69 seconds
- **Settling Time**: 8.0 seconds
- **Final Position**: 12.593 cm

#### Test Case 2: Setpoint = 20.292 cm

- **Steady-State Error**: 0.121 cm (0.60%)
- **Percentage Overshoot**: 6.55%
- **Peak Time**: 0.35 seconds
- **Settling Time**: 9.1 seconds
- **Final Position**: 20.413 cm

#### Test Case 3: Setpoint = 7.968 cm

- **Steady-State Error**: -0.009 cm (-0.11%)
- **Percentage Overshoot**: 79.20%
- **Peak Time**: 0.56 seconds
- **Settling Time**: 13.26 seconds
- **Final Position**: 7.959 cm

### System Specifications

- **Control Frequency**: ~33 Hz (30ms loop time)
- **Error Threshold**: ±3.5 mm
- **Sensor Accuracy**: ±2 mm (with 50-sample averaging)
- **Servo Response Time**: ~0.04 seconds
- **Beam Length**: 30 cm
- **Operating Range**: 6-25 cm

## 🐛 Troubleshooting

### Car Won't Balance

**Symptoms**: Car oscillates wildly or falls off beam

**Solutions**:
1. Check PID gains - reduce Kp if oscillating
2. Verify servo neutral position (should be 90° when beam is level)
3. Ensure car has sufficient weight
4. Check for mechanical friction in beam/servo
5. Verify sensor alignment

### Erratic Sensor Readings

**Symptoms**: Position jumps randomly, inconsistent values

**Solutions**:
1. Ensure reflective board is flat and perpendicular to sensor
2. Check sensor wiring and power supply
3. Increase NUM_SAMPLES for more averaging
4. Keep sensor wires away from servo wires (EMI)
5. Use external power supply for servo

### Servo Jittering

**Symptoms**: Servo makes noise, vibrates excessively

**Solutions**:
1. Use external 5V power supply (≥2A)
2. Reduce Kd (derivative gain)
3. Increase derivative filter coefficient
4. Check for loose mechanical connections

### Car Gets Stuck

**Symptoms**: Car stays at one position despite error

**Solutions**:
- System has automatic stuck detection (3-second timeout)
- If not recovering, check:
  1. Servo torque is sufficient
  2. Beam is properly lubricated
  3. Car wheels are clean and rolling freely
  4. Stuck angle parameters (STUCK_ANGLE_LOW/HIGH)

### No Serial Output

**Checklist**:
- [ ] Arduino powered and code uploaded successfully
- [ ] Serial Monitor open with correct baud rate (9600)
- [ ] Correct COM port selected
- [ ] USB cable supports data transfer (not charge-only)

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

### Development Setup

1. Fork the repository
2. Create a feature branch:
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. Make your changes and test thoroughly
4. Commit with descriptive messages:
   ```bash
   git commit -m "Add: amazing feature description"
   ```
5. Push to your fork:
   ```bash
   git push origin feature/amazing-feature
   ```
6. Open a Pull Request

### Contribution Ideas

- [ ] Add Kalman filtering for sensor fusion
- [ ] Implement adaptive PID tuning
- [ ] Create mobile app interface (Bluetooth control)
- [ ] Design PCB for cleaner assembly
- [ ] Add multiple setpoint tracking
- [ ] Implement cascade control strategy
- [ ] Create 3D-printable parts (STL files)
- [ ] Add video tutorials

### Code Style

- Follow existing naming conventions
- Add comments for complex logic
- Test on actual hardware before submitting
- Update documentation for new features

## 👥 Team

This project was developed by Group 03, Section A1:

| Name | Student ID | Role |
|------|------------|------|
| **Sayba Kamal Orni** | 2006009 | Hardware Implementation, Documentation |
| **Soumik Saha** | 2006011 | Coding, Hardware Implementation |
| **Adith Saha Dipro** | 2006018 | System Modeling, Documentation |
| **Md. Adib Rahman** | 2006024 | Simulation, Debugging |
| **Nittya Ananda Biswas** | 2006025 | System Modeling, Debugging |
| **Khondakar Ashik Shahriar** | 2006026 | Coding, Simulation |

**Course**: EEE 318 - Control Systems Laboratory (January 2024)  
**Institution**: Bangladesh University of Engineering and Technology (BUET)  
**Department**: Electrical and Electronic Engineering

### Course Instructors

- Shafin Bin Hamid, Lecturer
- Md. Meherab Hossain, Part-Time Lecturer

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2024 Soumik Saha and Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction...
```

## 🙏 Acknowledgments

- Classical control theory fundamentals from **Modern Control Engineering** by Katsuhiko Ogata
- Arduino community for servo and sensor libraries
- BUET Department of Electrical and Electronic Engineering for lab facilities
- Course instructors for guidance and support

## 📚 References

1. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.
2. Maalini, P. V. M., et al. (2016). "Modelling and control of ball and beam system using PID controller." *IEEE ICACCCT*.
3. Taifour Ali, A., et al. (2017). "Design and Implementation of Ball and Beam System Using PID Controller." *Automatic Control and Information Sciences*, 3(1), 1-4.

## 📞 Contact

For questions, suggestions, or collaboration opportunities:

- **GitHub**: [@soumik-saha](https://github.com/soumik-saha)  
- **Email**: [2006011@eee.buet.ac.bd
- ]
- **Project Link**: [https://github.com/soumik-saha/ball-beam-pid-controller](https://github.com/soumik-saha/ball-beam-pid-controller)

---

<p align="center">
  <b>⭐ Star this repository if you find it helpful!</b>
</p>

<p align="center">
  Made with ❤️ by BUET EEE Students
</p>

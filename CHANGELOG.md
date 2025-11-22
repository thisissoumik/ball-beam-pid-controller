# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2024-12-27

### Added
- Initial release of Ball and Beam PID Controller
- Complete Arduino code with PID control algorithm
- Ziegler-Nichols tuned PID parameters (Kp=8.4, Ki=3.73, Kd=8.0)
- Adaptive servo control with variable speed based on error magnitude
- Stuck detection and automatic recovery mechanism
- 50-sample averaging for robust position measurement
- Digital filtering on setpoint, position, and derivative terms
- Anti-windup protection for integral term
- CSV data logging via Serial Monitor
- Comprehensive documentation and README
- MATLAB analysis scripts for system modeling
- MATLAB plotting scripts for experimental data visualization
- Circuit diagram and wiring instructions
- Project report (38 pages, complete technical documentation)
- Quick start guide for rapid deployment
- Contributing guidelines
- MIT License

### Features
- **PID Control**: Tunable proportional-integral-derivative controller
- **Sensor Fusion**: Ultrasonic distance measurement with averaging
- **Adaptive Control**: Error-dependent servo speed (3° vs 5° steps)
- **Fault Tolerance**: Automatic stuck detection after 3 seconds
- **Data Logging**: Real-time CSV output for analysis
- **Safety**: Angle saturation limits (±30°) and physical barriers

### Hardware Support
- Arduino Uno/Nano (ATmega328P)
- HC-SR04 Ultrasonic Sensors (2x)
- MG996R Servo Motor (or compatible)
- Custom mechanical assembly

### Documentation
- Main README with complete instructions
- Arduino-specific README
- MATLAB script documentation
- Full technical report
- Quick start guide
- Contributing guidelines

## [Future Releases]

### Planned for v1.1.0
- [ ] Kalman filter implementation for sensor fusion
- [ ] Web-based dashboard for real-time monitoring
- [ ] Parameter auto-tuning feature
- [ ] Mobile app (Bluetooth control)
- [ ] 3D-printable enclosure STL files

### Planned for v1.2.0
- [ ] Adaptive PID controller
- [ ] Multiple setpoint tracking
- [ ] Cascade control implementation
- [ ] State-space controller option
- [ ] PCB design files

### Planned for v2.0.0
- [ ] Machine learning-based controller
- [ ] Multi-ball balancing
- [ ] Remote monitoring via IoT
- [ ] Advanced visualization dashboard
- [ ] Complete educational toolkit

## Version History

- **v1.0.0** (2024-12-27): Initial public release

---

For detailed changes between versions, see the [commit history](https://github.com/soumik-saha/ball-beam-pid-controller/commits/main).

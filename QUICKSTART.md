# Quick Start Guide

## 🚀 Get Running in 15 Minutes

### Step 1: Hardware Assembly (5 min)

1. Mount servo motor to base
2. Attach beam to servo lever arm
3. Place sensors:
   - Sensor 1 at base (measures setpoint)
   - Sensor 2 above beam (measures car position)
4. Attach reflective board to front of car

### Step 2: Wiring (5 min)

```
Connections:
Arduino Pin 5  ← Sensor 1 Echo
Arduino Pin 6  → Sensor 1 Trig
Arduino Pin 9  → Servo Signal (Orange)
Arduino Pin 10 → Sensor 2 Trig  
Arduino Pin 11 ← Sensor 2 Echo

5V  → All VCC pins
GND → All GND pins
```

**⚠️ Important**: Use external 5V 2A power supply for servo!

### Step 3: Upload Code (3 min)

```bash
# Clone repository
git clone https://github.com/soumik-saha/ball-beam-pid-controller.git
cd ball-beam-pid-controller

# Open Arduino IDE
# File → Open → arduino/ball_beam_controller.ino
# Tools → Board → Arduino Uno
# Tools → Port → [Your Port]
# Click Upload ↑
```

### Step 4: Test (2 min)

1. Open Serial Monitor (9600 baud)
2. Place car on beam
3. Watch it balance! 🎉

## 📊 View Data

### Option 1: Serial Plotter (Built-in)
```
Tools → Serial Plotter
```

### Option 2: MATLAB
```matlab
% Save Serial Monitor data as 'serial_data.csv'
cd matlab/
plot_experimental_data
```

### Option 3: Python
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('serial_data.csv', 
                 names=['Time', 'Setpoint', 'Position'])
df['Time'] /= 1000

plt.plot(df['Time'], df['Setpoint'], 'r--', label='Setpoint')
plt.plot(df['Time'], df['Position'], 'b-', label='Position')
plt.legend()
plt.show()
```

## 🎛️ Tuning (If Needed)

### Too Much Oscillation?
```cpp
const double KP = 6.0;    // Decrease Kp
const double KD = 10.0;   // Increase Kd
```

### Too Slow?
```cpp
const double KP = 12.0;   // Increase Kp
```

### Steady-State Error?
```cpp
const double KI = 5.0;    // Increase Ki
```

## 🐛 Common Issues

| Problem | Solution |
|---------|----------|
| Servo doesn't move | Check power supply (need 5V 2A) |
| Car won't balance | Reduce Kp, check mechanical friction |
| Erratic readings | Add reflective board, increase NUM_SAMPLES |
| No serial output | Check baud rate (9600) |

## 📖 Next Steps

- Read full [README.md](README.md) for detailed information
- Check [CONTRIBUTING.md](CONTRIBUTING.md) to contribute
- View [Project_Report.pdf](docs/Project_Report.pdf) for theory

## 🆘 Need Help?

- Check [Troubleshooting](README.md#troubleshooting) section
- Open an [Issue](https://github.com/soumik-saha/ball-beam-pid-controller/issues)
- Contact: soumik.saha@example.com

---

**Happy Balancing! 🎯**

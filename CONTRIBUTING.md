# Contributing to Ball and Beam PID Controller

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## 🌟 Ways to Contribute

- 🐛 Report bugs
- 💡 Suggest new features
- 📝 Improve documentation
- 🔧 Submit bug fixes
- ✨ Add new features
- 🎨 Improve hardware design
- 📊 Add visualization tools

## 🐛 Reporting Bugs

When reporting bugs, please include:

### Bug Report Template

```markdown
**Description**
A clear description of the bug

**Steps to Reproduce**
1. Step one
2. Step two
3. ...

**Expected Behavior**
What should happen

**Actual Behavior**
What actually happens

**Hardware Setup**
- Arduino board: [e.g., Arduino Uno]
- Servo model: [e.g., MG996R]
- Sensor model: [e.g., HC-SR04]
- Power supply: [e.g., 5V 2A]

**Code Configuration**
- KP: [value]
- KI: [value]
- KD: [value]
- Other parameters: [if modified]

**Serial Output**
```
Paste relevant serial output here
```

**Additional Context**
Any other relevant information
```

## 💡 Suggesting Features

Feature requests are welcome! Please include:

- **Description**: Clear explanation of the feature
- **Use Case**: Why this feature would be useful
- **Implementation Ideas**: How it might be implemented (optional)
- **Alternatives**: Alternative solutions you've considered

## 🔧 Pull Request Process

### Before You Start

1. Check existing issues to avoid duplicate work
2. Open an issue to discuss major changes
3. Fork the repository

### Development Workflow

1. **Create a Branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make Changes**:
   - Follow existing code style
   - Add comments for complex logic
   - Test on actual hardware
   - Update documentation

3. **Commit Changes**:
   ```bash
   git add .
   git commit -m "Add: description of changes"
   ```
   
   **Commit Message Prefixes**:
   - `Add:` - New features
   - `Fix:` - Bug fixes
   - `Update:` - Modifications
   - `Remove:` - Deletions
   - `Docs:` - Documentation only
   - `Refactor:` - Code restructuring
   - `Test:` - Adding tests

4. **Push to Fork**:
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Create Pull Request**:
   - Provide clear description
   - Reference related issues
   - Include test results
   - Add screenshots/videos if applicable

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Code refactoring
- [ ] Performance improvement

## Testing
- [ ] Tested on actual hardware
- [ ] Multiple test scenarios verified
- [ ] No regression in existing functionality

## Hardware Tested
- Arduino: [model]
- Servo: [model]
- Sensors: [model]

## Related Issues
Fixes #(issue number)

## Screenshots/Videos
[If applicable]

## Additional Notes
Any other relevant information
```

## 📝 Code Style Guidelines

### Arduino C++

```cpp
// Constants: UPPER_CASE
const int SERVO_PIN = 9;

// Variables: camelCase
float ballPosition = 0.0;

// Functions: camelCase
void calculatePID() {
    // Implementation
}

// Single-line comments for brief explanations

/**
 * Multi-line comments for function documentation
 * @param angle The servo angle in degrees
 * @return The calculated position
 */
```

### Formatting

- **Indentation**: 4 spaces (no tabs)
- **Line Length**: Maximum 100 characters
- **Braces**: Opening brace on same line
- **Spacing**: Space after keywords (`if (condition)`)

### Organization

- Group related constants
- Define pins at top
- Organize functions logically
- Add section separators

### Documentation

- Update README for user-facing changes
- Add inline comments for complex algorithms
- Include example code
- Explain parameter effects

## 🧪 Testing Guidelines

### Hardware Testing

Test these scenarios before submitting:

1. **Basic Functionality**:
   - Ball reaches and maintains setpoint
   - Servo responds smoothly
   - No erratic behavior

2. **Edge Cases**:
   - Ball at extreme positions
   - Large disturbances
   - Recovery after saturation

3. **Different Configurations**:
   - Various PID parameters
   - Different ball weights
   - Alternative servo speeds

### Data Validation

- Record serial output for test runs
- Plot setpoint vs. position
- Measure settling time and overshoot
- Check for steady-state error

## 📚 Documentation Standards

### README Updates

When adding features, update:
- Features list
- Hardware requirements (if changed)
- Configuration section
- Usage instructions
- Troubleshooting

### Code Comments

- Explain "why" not just "what"
- Document assumptions
- Note limitations
- Reference formulas/sources

## 🎯 Priority Areas

Contributions especially valuable in:

1. **Advanced Control**:
   - Adaptive PID tuning
   - Cascade control
   - State-space control
   - Fuzzy logic integration

2. **Visualization**:
   - Real-time plotting tools
   - Web interface
   - Mobile app (Bluetooth)

3. **Hardware**:
   - 3D printable parts
   - PCB design
   - Alternative sensor options

4. **Documentation**:
   - Video tutorials
   - Assembly guide with photos
   - Troubleshooting flowcharts

5. **Analysis Tools**:
   - Python scripts for data analysis
   - MATLAB/Simulink models
   - Performance benchmarking

## ❓ Questions

If you have questions:

1. Check existing issues and discussions
2. Create new issue with `question` label
3. Reach out to maintainers

## 🙏 Recognition

All contributors will be recognized in the project. Significant contributions may be highlighted in the README.

## 📜 Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Focus on what's best for the project
- Help others learn and grow

Thank you for contributing! 🎉

# Self-Balancing Robot

A two-wheeled self-balancing robot using Arduino Nano, MPU6050 gyroscope/accelerometer, and PID control algorithm.

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Wiring Diagram](#wiring-diagram)
- [Installation](#installation)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
- [Troubleshooting](#troubleshooting)
- [Tuning Guide](#tuning-guide)
- [Serial Monitor Output](#serial-monitor-output)

## 🔧 Hardware Requirements

### Essential Components:
- **Arduino Nano** (ATmega328P)
- **MPU6050** - 6-axis gyroscope and accelerometer module
- **L298N Motor Driver** (or similar H-bridge driver)
- **2x DC Motors** with gearbox (recommended: 100-300 RPM)
- **2x Wheels** (diameter: 65-100mm recommended)
- **Battery** (7.4V - 12V LiPo or Li-ion recommended)
- **Chassis** - Custom or 3D printed
- **Jumper Wires**
- **Switch** (optional but recommended)

### Optional Components:
- Battery voltage indicator
- Power switch
- Mounting brackets
- Breadboard or PCB

## 💻 Software Requirements

- **PlatformIO** (recommended) or Arduino IDE
- **VS Code** (if using PlatformIO)

### Required Libraries:
- `I2Cdev` - I2C communication library
- `MPU6050` - MPU6050 sensor library with DMP
- `PID_v1` - PID control library

These are automatically installed via PlatformIO dependencies.

## 🔌 Wiring Diagram

### MPU6050 to Arduino Nano:
| MPU6050 Pin | Arduino Nano Pin |
|-------------|------------------|
| VCC         | 5V               |
| GND         | GND              |
| SCL         | A5               |
| SDA         | A4               |
| INT         | D2               |

### L298N Motor Driver to Arduino Nano:
| L298N Pin | Arduino Nano Pin | Description        |
|-----------|------------------|--------------------|
| IN1       | D6               | Motor 1 Forward    |
| IN2       | D9               | Motor 1 Reverse    |
| IN3       | D10              | Motor 2 Forward    |
| IN4       | D11              | Motor 2 Reverse    |
| ENA       | D5*              | Motor 1 Enable     |
| ENB       | D3*              | Motor 2 Enable     |
| 12V       | Battery +        | Motor Power Supply |
| GND       | Battery - & Arduino GND | Common Ground |

**Note:** If your L298N has ENA/ENB jumpers installed, you may not need pins D5 and D3. Remove jumpers if you want PWM speed control on these pins.

### Power Supply:
- Motor Driver: Connect to 7.4V - 12V battery
- Arduino: Can be powered via USB during testing, or from motor driver's 5V output (if available)
- **Important:** Always connect common ground between Arduino and motor driver

## 📥 Installation

### Using PlatformIO (Recommended):

1. **Clone or download this repository**
   ```bash
   git clone <your-repo-url>
   cd 251023-121233-nanoatmega328
   ```

2. **Open in VS Code with PlatformIO**
   ```bash
   code .
   ```

3. **Build the project**
   - Click the PlatformIO icon in the sidebar
   - Select "Build" or press `Ctrl+Alt+B`

4. **Upload to Arduino**
   - Connect Arduino Nano via USB
   - Click "Upload" or press `Ctrl+Alt+U`

### Using Arduino IDE:

1. **Install required libraries:**
   - Go to: Sketch → Include Library → Manage Libraries
   - Search and install:
     - `I2Cdev` by Jeff Rowberg
     - `MPU6050` by Jeff Rowberg
     - `PID` by Brett Beauregard

2. **Open the sketch:**
   - Open `src/SelfBalancingRobot.ino`

3. **Select board:**
   - Tools → Board → Arduino Nano
   - Tools → Processor → ATmega328P (Old Bootloader) *if using clone*

4. **Upload**

## ⚙️ Configuration

### 1. Calibrate MPU6050 Gyro Offsets

The gyro offsets compensate for manufacturing imperfections in the sensor.

**Option A: Use IMU_Zero Calibration Sketch**
1. Download from: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero
2. Upload to Arduino
3. Open Serial Monitor (115200 baud)
4. Wait ~10 minutes for calibration
5. Copy the offset values and update in code:

```cpp
mpu.setXGyroOffset(-479);  // Replace with your values
mpu.setYGyroOffset(84);
mpu.setZGyroOffset(15);
mpu.setZAccelOffset(1638);
```

**Option B: Manual Calibration**
1. Place robot on flat surface
2. Upload code with offsets set to 0
3. Observe sensor drift in Serial Monitor
4. Adjust offsets until values are close to 0 when stationary

### 2. Find the Setpoint (Balance Point)

The setpoint is the angle where your robot balances perfectly.

```cpp
double setpoint = 182;  // Adjust this value
```

**How to find it:**
1. Assemble your robot completely
2. Upload the code
3. Open Serial Monitor (115200 baud)
4. Hold the robot vertically (balanced position)
5. Note the `input` value displayed
6. Update `setpoint` to match this value
7. Re-upload

### 3. Tune PID Values

Start with conservative values and tune in order:

```cpp
double Kp = 15;   // Start here
double Kd = 0.9;  // Tune second
double Ki = 140;  // Tune last
```

See the [Tuning Guide](#tuning-guide) section for detailed instructions.

## 🧠 How It Works

### System Overview:

1. **Sensor Reading**: MPU6050 measures the robot's tilt angle using its gyroscope and accelerometer
2. **DMP Processing**: The Digital Motion Processor (DMP) fuses sensor data to calculate accurate orientation
3. **PID Control**: Compares current angle with setpoint and calculates correction
4. **Motor Control**: Drives motors forward/backward to maintain balance

### Control Flow:

```
MPU6050 → DMP → Quaternion → Pitch Angle → PID Controller → Motor Speed → Balance
```

### Key Variables:

- **`input`**: Current pitch angle in degrees (from sensor)
- **`output`**: Motor speed correction (-255 to 255)
- **`setpoint`**: Target balance angle in degrees
- **`Kp`**: Proportional gain (reacts to current error)
- **`Ki`**: Integral gain (eliminates steady-state error)
- **`Kd`**: Derivative gain (dampens oscillations)

### Interrupt System:

The MPU6050 uses **Digital Pin 2** (Interrupt 0) to signal when new data is available. This ensures real-time response without polling delays.

## 🔍 Troubleshooting

### Motors Don't Turn:

1. **Check Enable Pins**: Add enable pin configuration if missing:
   ```cpp
   pinMode(5, OUTPUT);
   pinMode(3, OUTPUT);
   digitalWrite(5, HIGH);
   digitalWrite(3, HIGH);
   ```

2. **Verify Power Supply**: 
   - Motors need separate power (7-12V battery)
   - Check battery is charged
   - Measure voltage with multimeter

3. **Check Angle Range**:
   - Robot only balances when angle is between 150-200°
   - Add debug output to verify angle readings

4. **Test Motors Manually**:
   ```cpp
   // Add to setup() for testing
   analogWrite(6, 150);
   analogWrite(10, 150);
   delay(2000);
   ```

### MPU6050 Connection Failed:

1. Check wiring (SDA → A4, SCL → A5)
2. Verify 5V and GND connections
3. Try different I2C pull-up resistors (not usually needed)
4. Check if MPU6050 address is 0x68 (default)

### Robot Falls Over Immediately:

1. Wrong setpoint - recalibrate
2. PID values too aggressive - reduce Kp
3. Motors spinning wrong direction - swap motor wires
4. Battery voltage too low

### FIFO Overflow Errors:

- Loop is running too slowly
- Reduce Serial.print() frequency
- Optimize code execution time

### Robot Oscillates Uncontrollably:

1. Kp is too high - reduce by 50%
2. Increase Kd to dampen oscillations
3. Check mechanical issues (loose wheels, wobbly frame)

## 📊 Tuning Guide

### Step-by-Step PID Tuning:

#### Phase 1: Proportional (Kp)
1. Set: `Kp = 20`, `Kd = 0`, `Ki = 0`
2. Upload and test
3. Gradually increase Kp until robot starts oscillating
4. Reduce Kp by 20-30%
5. Typical range: 15-100

#### Phase 2: Derivative (Kd)
1. Keep Kp from Phase 1
2. Set `Kd = 0.5`
3. Gradually increase Kd until oscillations are smooth
4. Too high = sluggish, too low = oscillates
5. Typical range: 0.5-2.0

#### Phase 3: Integral (Ki)
1. Keep Kp and Kd from previous phases
2. Set `Ki = 50`
3. Increase if robot drifts over time
4. Too high = instability
5. Typical range: 50-200

### Other Tunable Parameters:

```cpp
// Balance angle range (degrees from setpoint)
if (input > 150 && input < 200)  // Adjust range as needed

// PID output limits (motor speed)
pid.SetOutputLimits(-255, 255);  // Reduce if motors too powerful

// PID sample time (milliseconds)
pid.SetSampleTime(10);  // Usually don't need to change
```

### Signs of Good Tuning:
- ✅ Robot balances smoothly without excessive wobbling
- ✅ Recovers quickly from small pushes
- ✅ Doesn't drift forward/backward over time
- ✅ Smooth motor speed changes

## 📡 Serial Monitor Output

Set baud rate to **115200**.

### During Setup:
```
Initializing I2C devices...
Testing device connections...
MPU6050 connection successful
Enabling DMP...
Enabling interrupt detection (Arduino external interrupt 0)...
DMP ready! Waiting for first interrupt...
```

### During Operation:
```
182.45 =>0.00S
181.23 =>15.67F
183.89 =>-23.45R
175.00 =>0.00S
```

**Format**: `[angle] => [motor_speed] [state]`

- **First number**: Current pitch angle
- **Second number**: PID output (motor speed)
- **Letter**:
  - `S` = Stop (motors off, outside balance range)
  - `F` = Forward (correcting forward tilt)
  - `R` = Reverse (correcting backward tilt)

### Error Messages:
- `DMP Initialization failed (code 1)` - Memory load failed
- `DMP Initialization failed (code 2)` - DMP config failed
- `FIFO overflow!` - Data processing too slow

## 📝 Code Structure

```
src/
└── SelfBalancingRobot.ino    # Main code file

Libraries Used:
- I2Cdev                       # I2C communication
- MPU6050_6Axis_MotionApps20   # MPU6050 with DMP
- PID_v1                       # PID controller

Key Functions:
- setup()                      # Initialize hardware
- loop()                       # Main control loop
- dmpDataReady()              # Interrupt handler
- Forward()                    # Motor forward control
- Reverse()                    # Motor backward control
- Stop()                       # Stop motors
```

## 🎯 Project Specifications

- **Platform**: Arduino Nano (ATmega328P)
- **Framework**: Arduino
- **Serial Speed**: 115200 baud
- **PWM Frequency**: Default Arduino (~490Hz on pins 6,9,10,11)
- **Control Loop**: Interrupt-driven, ~10ms sample time
- **Sensor Update Rate**: ~200Hz (MPU6050 DMP)

## 🤝 Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## 📚 References

- [MPU6050 Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
- [PID Tuning Guide](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)

## 📄 License

This project is open source. Feel free to use and modify for your own projects.

## ⚠️ Safety Notes

1. **Test motors separately** before full assembly
2. **Use appropriate battery protection** (BMS for Li-ion/LiPo)
3. **Start with low PID values** to prevent violent movements
4. **Have an emergency stop method** (power switch)
5. **Test in open area** away from obstacles
6. **Secure all wiring** to prevent shorts

---

**Happy Building! 🤖**

For questions or issues, please open an issue on the repository.

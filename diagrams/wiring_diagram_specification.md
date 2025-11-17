# RoboSort Wiring Diagram Documentation

## Professional Wiring Schematic Standards

**Document Information:**
- **Document Title:** RoboSort System Wiring Diagram
- **Document Number:** RBS-WD-001
- **Revision:** 1.0
- **Date:** November 17, 2025
- **Author:** RoboSort Development Team
- **Standards Compliance:** IEEE 315-1975, IEC 60617

---

## 1. SYSTEM OVERVIEW

### 1.1 System Architecture
The RoboSort system employs a distributed architecture with:
- **High-Level Processing:** Raspberry Pi 4B for AI vision and decision making
- **Low-Level Control:** Arduino Mega 2560 for real-time hardware control
- **Power Distribution:** Centralized 12V DC supply with regulated outputs
- **Communication:** USB Serial interface at 9600 baud

### 1.2 Power Specifications
- **Input Voltage:** 12V DC ±5%
- **Input Current:** 3A maximum
- **Output Voltages:** 5V (2A), 12V (1.5A)
- **Protection:** Fused outputs, overcurrent protection

---

## 2. COMPONENT PINOUTS

### 2.1 Arduino Mega 2560 Pin Configuration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ARDUINO MEGA 2560 PINOUT                           │
│                                                                             │
│  POWER PINS:                                                               │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │ Pin         │ Function    │ Voltage     │ Current     │ Connection  │    │
│  ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│  │ VIN         │ Power Input │ 7-12V DC    │ 1A          │ Power Supply │    │
│  │ 5V          │ Regulated   │ 5V DC       │ 800mA       │ Sensors      │    │
│  │ 3.3V        │ Regulated   │ 3.3V DC     │ 150mA       │ Logic        │    │
│  │ GND         │ Ground      │ 0V          │ -           │ Common       │    │
│  └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
│                                                                             │
│  DIGITAL I/O PINS (PWM CAPABLE):                                           │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │ Pin         │ Function    │ Component   │ Signal      │ Notes       │    │
│  ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│  │ 3           │ PWM Output  │ Servo 1     │ Base        │ 0-180°       │    │
│  │ 4           │ PWM Output  │ Servo 2     │ Shoulder    │ 0-180°       │    │
│  │ 5           │ PWM Output  │ Servo 3     │ Elbow       │ 0-180°       │    │
│  │ 6           │ PWM Output  │ Servo 4     │ Wrist       │ 0-180°       │    │
│  │ 7           │ PWM Output  │ Servo 5     │ Gripper     │ 0-90°        │    │
│  │ 8           │ Digital Out │ Stepper     │ STEP        │ Pulse train   │    │
│  │ 9           │ Digital Out │ Stepper     │ DIR         │ Direction     │    │
│  │ 10          │ Digital Out │ Stepper     │ ENABLE      │ Active Low    │    │
│  │ 11          │ Digital Out │ Stepper     │ MS1         │ Microstep     │    │
│  │ 12          │ Digital Out │ Stepper     │ MS2         │ Microstep     │    │
│  │ 13          │ Digital Out │ Stepper     │ MS3         │ Microstep     │    │
│  └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
│                                                                             │
│  ANALOG INPUT PINS:                                                        │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │ Pin         │ Function    │ Component   │ Range       │ Resolution  │    │
│  ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│  │ A0          │ Analog In   │ Reserved    │ 0-5V        │ 10-bit       │    │
│  │ A1          │ Analog In   │ Reserved    │ 0-5V        │ 10-bit       │    │
│  │ A2          │ Analog In   │ Reserved    │ 0-5V        │ 10-bit       │    │
│  │ A3          │ Analog In   │ Reserved    │ 0-5V        │ 10-bit       │    │
│  │ A4          │ Analog In   │ I2C SDA     │ 0-5V        │ 10-bit       │    │
│  │ A5          │ Analog In   │ I2C SCL     │ 0-5V        │ 10-bit       │    │
│  └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
│                                                                             │
│  SERIAL COMMUNICATION:                                                      │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │ Serial      │ TX Pin      │ RX Pin      │ Baud Rate   │ Purpose     │    │
│  ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│  │ Serial0     │ 1 (TX0)     │ 0 (RX0)     │ 9600        │ USB/RPi     │    │
│  │ Serial1     │ 18 (TX1)    │ 19 (RX1)    │ Reserved    │ Future Use  │    │
│  │ Serial2     │ 16 (TX2)    │ 17 (RX2)    │ 230400      │ LD06 LIDAR  │    │
│  │ Serial3     │ 14 (TX3)    │ 15 (RX3)    │ Reserved    │ Future Use  │    │
│  └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
│                                                                             │
│  DIGITAL I/O PINS (ULTRASONIC):                                            │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │ Pin         │ Function    │ Component   │ Signal      │ Notes       │    │
│  ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│  │ 22          │ Digital Out │ Ultrasonic  │ TRIGGER     │ 10μs pulse   │    │
│  │ 23          │ Digital In  │ Ultrasonic  │ ECHO        │ Pulse width  │    │
│  └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. WIRING DIAGRAM SPECIFICATIONS

### 3.1 Power Distribution Network

```
POWER SUPPLY CHAIN DIAGRAM:
┌─────────────────┐
│ 12V DC Supply   │
│ (3A, 36W)       │
└────────┬────────┘
         │
    ┌────▼────┐
    │ Fuse    │ 2A
    │ (F1)    │
    └────┬────┘
         │
    ┌────▼────┐
    │Arduino  │ VIN (7-12V)
    │Mega     │
    │2560     │
    └────┬────┘
         │
    ┌────▼────┐
    │ 5V Reg  │ LM7805
    │ (U1)    │
    └────┬────┘
         │
    ┌────▼────┐
    │ Fuse    │ 2A
    │ (F2)    │
    └────┬────┘
         │
   ┌─────┼─────┐
   │           │
┌──▼──┐    ┌───▼──┐
│Servo│    │LIDAR │
│Motors│   │LD06  │
│(5V) │    │(5V)  │
└─────┘    └──────┘
```

### 3.2 Signal Flow Diagram

```
SIGNAL FLOW ARCHITECTURE:
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Raspberry Pi  │    │   Arduino Mega  │    │   Hardware      │
│   (Master)      │    │   (Controller)  │    │   (Actuators)   │
├─────────────────┤    ├─────────────────┤    ├─────────────────┤
│ • USB Camera    │    │ • PWM Outputs   │    │ • Servo Motors  │
│ • AI Processing │    │ • Digital I/O   │    │ • Stepper Motor │
│ • Decision      │    │ • Serial Comm   │    │ • Sensors       │
│ • Commands      │    │ • ADC Inputs    │    │ • Indicators    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                     │                     │
          └─────────┬───────────┼─────────┬───────────┘
                    │           │         │
               ┌────▼────┐ ┌────▼────┐ ┌──▼──┐
               │ USB     │ │ Serial  │ │ PWM │
               │ Cable   │ │ Ports   │ │ Bus │
               │ (Data)  │ │ (Data)  │ │ (Ctrl)
               └─────────┘ └─────────┘ └─────┘
```

### 3.3 Component Interconnection Details

#### 3.3.1 Servo Motor Connections
```
SERVO MOTOR WIRING STANDARD (MG996R):
┌─────────────────────────────────────────────────────────────┐
│ Position: Base (Servo 1) - Pin 3                            │
├─────────────────────────────────────────────────────────────┤
│ Arduino Pin 3 (PWM) ────► Servo Signal (Orange/Yellow)     │
│ Arduino 5V ─────────────► Servo Power (Red)                 │
│ Arduino GND ────────────► Servo Ground (Brown/Black)        │
│                                                             │
│ Electrical Specifications:                                  │
│ • Operating Voltage: 4.8-6.0V                              │
│ • Current Draw: 500mA (stall), 100mA (idle)                │
│ • Signal Pulse: 500-2500μs (0-180°)                        │
│ • Update Rate: 50Hz                                        │
│ • Torque: 9.4kg·cm (4.8V), 11kg·cm (6V)                    │
└─────────────────────────────────────────────────────────────┘
```

#### 3.3.2 Stepper Motor Driver (A4988)
```
STEPPER DRIVER CONNECTIONS:
┌─────────────────────────────────────────────────────────────┐
│ A4988 Stepper Driver Module                                 │
├─────────────────────────────────────────────────────────────┤
│ Arduino Pin 8 ─────► STEP (Pulse Input)                     │
│ Arduino Pin 9 ─────► DIR (Direction)                        │
│ Arduino Pin 10 ────► ENABLE (Active Low)                    │
│ Arduino Pin 11 ────► MS1 (Microstep 1)                      │
│ Arduino Pin 12 ────► MS2 (Microstep 2)                      │
│ Arduino Pin 13 ────► MS3 (Microstep 3)                      │
│                                                             │
│ Power Connections:                                          │
│ 12V Supply ─────────► VMOT (Motor Power)                    │
│ Arduino 5V ─────────► VDD (Logic Power)                     │
│ Common GND ─────────► GND (Logic & Motor Ground)            │
│                                                             │
│ Motor Coil Connections:                                     │
│ Stepper Coil A1 ────► 1A                                    │
│ Stepper Coil A2 ────► 1B                                    │
│ Stepper Coil B1 ────► 2A                                    │
│ Stepper Coil B2 ────► 2B                                    │
│                                                             │
│ Configuration Jumpers:                                      │
│ MS1-MS3: Microstepping mode (see table below)               │
└─────────────────────────────────────────────────────────────┘

MICROSTEPPING MODES:
┌─────────┬──────┬──────┬──────┬─────────────┐
│ Mode    │ MS1  │ MS2  │ MS3  │ Steps/Rev   │
├─────────┼──────┼──────┼──────┼─────────────┤
│ Full    │ Low  │ Low  │ Low  │ 200         │
│ Half    │ High │ Low  │ Low  │ 400         │
│ 1/4     │ Low  │ High │ Low  │ 800         │
│ 1/8     │ High │ High │ Low  │ 1600        │
│ 1/16    │ High │ High │ High │ 3200        │
└─────────┴──────┴──────┴──────┴─────────────┘
```

#### 3.3.3 LIDAR Sensor (LD06)
```
LD06 LIDAR CONNECTIONS:
┌─────────────────────────────────────────────────────────────┐
│ LD06 360° LIDAR Sensor                                      │
├─────────────────────────────────────────────────────────────┤
│ Arduino Pin 16 (TX2) ──► LIDAR RX (Receive Data)           │
│ Arduino Pin 17 (RX2) ──► LIDAR TX (Transmit Data)          │
│ Arduino 5V ─────────────► LIDAR VCC (Power)                 │
│ Arduino GND ────────────► LIDAR GND (Ground)                │
│                                                             │
│ Communication Specifications:                               │
│ • Interface: UART Serial                                    │
│ • Baud Rate: 230400 bps                                     │
│ • Data Format: 8N1 (8 data, no parity, 1 stop)              │
│ • Voltage: 4.8-5.2V                                        │
│ • Current: 150mA (typical), 200mA (max)                    │
│ • Range: 0.12-12m                                          │
│ • Angular Resolution: 1°                                   │
│ • Scan Rate: 5-15Hz                                        │
└─────────────────────────────────────────────────────────────┘
```

#### 3.3.4 Ultrasonic Sensor (HC-SR04)
```
HC-SR04 ULTRASONIC SENSOR:
┌─────────────────────────────────────────────────────────────┐
│ HC-SR04 Distance Sensor                                     │
├─────────────────────────────────────────────────────────────┤
│ Arduino Pin 22 ───────► TRIGGER (10μs pulse to start)       │
│ Arduino Pin 23 ───────► ECHO (pulse width = distance)       │
│ Arduino 5V ───────────► VCC (Power Supply)                  │
│ Arduino GND ──────────► GND (Ground)                        │
│                                                             │
│ Operating Specifications:                                   │
│ • Voltage: 5V DC                                           │
│ • Current: 15mA (idle), 60mA (active)                       │
│ • Range: 2cm - 400cm                                       │
│ • Resolution: 0.3cm                                        │
│ • Accuracy: ±3mm                                           │
│ • Trigger Pulse: 10μs minimum                              │
│ • Echo Pulse: 150μs - 25ms (proportional to distance)      │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. CABLE SPECIFICATIONS

### 4.1 Wire Gauge Requirements
```
WIRE GAUGE SPECIFICATIONS:
┌─────────────────┬─────────────┬─────────────┬─────────────┐
│ Application     │ Wire Gauge  │ Max Length  │ Max Current │
├─────────────────┼─────────────┼─────────────┼─────────────┤
│ Power Supply    │ 18 AWG      │ 2m          │ 3A          │
│ Servo Signals   │ 22 AWG      │ 1m          │ 100mA       │
│ Stepper Power   │ 20 AWG      │ 1.5m        │ 1.5A        │
│ Sensor Signals  │ 24 AWG      │ 0.5m        │ 50mA        │
│ USB Data        │ 28 AWG      │ 2m          │ 500mA       │
└─────────────────┴─────────────┴─────────────┴─────────────┘
```

### 4.2 Connector Standards
- **USB:** USB 2.0 Type A to Type B cable (for RPi-Arduino connection)
- **Power:** 2.1mm DC barrel jack (center positive)
- **Servo:** 3-pin JST or Molex connectors (signal, power, ground)
- **Stepper:** 4-pin Dupont connectors for motor coils
- **Sensors:** Dupont jumpers for prototyping, JST-XH for production

---

## 5. SAFETY AND PROTECTION

### 5.1 Electrical Safety
- All power connections include appropriate fusing
- Ground connections are continuous and low-resistance
- No live conductors exposed in accessible areas
- Power supplies meet UL/CE safety standards

### 5.2 Component Protection
- Arduino inputs protected with clamping diodes
- Motor drivers include thermal shutdown protection
- Sensors include input filtering and ESD protection
- USB communication includes overcurrent protection

---

## 6. TESTING AND VERIFICATION

### 6.1 Pre-Power Tests
□ Visual inspection of all connections
□ Continuity testing of power and ground nets
□ Resistance measurements for shorts/opens
□ Component orientation verification

### 6.2 Power-On Tests
□ Voltage measurements at all test points
□ Current draw verification
□ Communication link establishment
□ Basic functionality testing

### 6.3 Functional Tests
□ Servo motor range of motion
□ Stepper motor direction control
□ Sensor data acquisition
□ Serial communication reliability

---

## 7. DOCUMENT REVISION HISTORY

| Revision | Date | Author | Changes |
|----------|------|--------|---------|
| 1.0 | 2025-11-17 | RoboSort Team | Initial release with complete wiring specifications |

---

## 8. REFERENCE DOCUMENTS

- Arduino Mega 2560 Datasheet
- A4988 Stepper Driver Datasheet
- LD06 LIDAR Technical Specifications
- HC-SR04 Ultrasonic Sensor Datasheet
- MG996R Servo Motor Specifications
- IEEE 315-1975 Graphic Symbols for Electrical Diagrams
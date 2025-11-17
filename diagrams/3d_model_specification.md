# RoboSort 3D Model Documentation

## Professional 3D Model Standards

**Document Information:**
- **Document Title:** RoboSort System 3D Models and Assemblies
- **Document Number:** RBS-3D-001
- **Revision:** 1.0
- **Date:** November 17, 2025
- **Author:** RoboSort Development Team
- **Software:** Fusion 360, Blender, FreeCAD
- **Standards:** ASME Y14.5-2018, ISO 1101:2017

---

## 1. SYSTEM OVERVIEW

### 1.1 3D Model Hierarchy
```
RoboSort_Assembly_v1.0/
├── Base_Platform/
│   ├── Main_Base.STL
│   ├── Motor_Mounts.STL
│   └── Cable_Management.STL
├── Robotic_Arm/
│   ├── Base_Segment.STL
│   ├── Shoulder_Segment.STL
│   ├── Elbow_Segment.STL
│   ├── Wrist_Segment.STL
│   └── Gripper.STL
├── Electronics_Enclosure/
│   ├── Raspberry_Pi_Mount.STL
│   ├── Arduino_Mount.STL
│   └── Power_Supply_Housing.STL
├── Sensor_Mounts/
│   ├── Camera_Mount.STL
│   ├── LIDAR_Mount.STL
│   └── Ultrasonic_Mount.STL
└── Waste_Bins/
    ├── Paper_Bin.STL
    ├── Plastic_Bin.STL
    └── Other_Bin.STL
```

### 1.2 Design Specifications
- **Units:** Millimeters (mm)
- **Tolerance:** ±0.1mm for general features, ±0.05mm for critical fits
- **Material:** PLA/ABS for prototyping, PETG/Nylon for production
- **Print Settings:** 0.2mm layer height, 3-5 shells, 20-40% infill
- **Assembly:** M3/M4 screws, heat-set inserts for threaded connections

---

## 2. MAIN BASE PLATFORM

### 2.1 Dimensions and Specifications

```
MAIN BASE PLATFORM SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│                              BASE PLATFORM                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ Overall Dimensions: 400mm × 300mm × 20mm                                   │
│                                                                             │
│ Key Features:                                                              │
│ • Mounting holes for robotic arm base (4× M4)                              │
│ • Arduino Mega mounting area (100mm × 60mm)                                │
│ • Power supply mounting area (80mm × 50mm)                                 │
│ • Cable routing channels (10mm wide × 5mm deep)                            │
│ • Leveling feet mounting (4× M5)                                           │
│ • Waste bin positioning guides                                             │
│                                                                             │
│ Material Thickness:                                                         │
│ • Base plate: 20mm                                                         │
│ • Ribs: 15mm (for strength)                                                │
│ • Mounting bosses: 12mm (for threaded inserts)                             │
│                                                                             │
│ Weight: 850g (estimated with infill)                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Detailed Views

#### Top View (Assembly)
```
TOP VIEW - BASE PLATFORM ASSEMBLY:
┌─────────────────────────────────────────────────────────────────────────────┐
│                           400mm                                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   Waste Input   │    │   Robotic Arm   │    │   Waste Bins    │         │
│  │     Area        │    │     Base        │    │   (Sorted)      │         │
│  │   150mm × 150mm │    │   Ø60mm         │    │   3 × 100mm     │         │
│  │                 │    │                 │    │   × 100mm       │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        ELECTRONICS BAY                             │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │   │
│  │  │ Raspberry Pi│    │  Arduino   │    │ Power Supply│             │   │
│  │  │   Mount     │    │   Mega     │    │   Housing   │             │   │
│  │  │ 90×60mm     │    │ 100×60mm   │    │  80×50mm    │             │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  300mm                                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Side View (Cross-Section)
```
SIDE VIEW - BASE PLATFORM CROSS-SECTION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Height: 150mm (total assembly)                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ Waste Input Area (50mm height)                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ Robotic Arm Assembly (300mm reach)                                 │   │
│  │ • Base Servo: 50mm height                                          │   │
│  │ • Shoulder: +100mm                                                 │   │
│  │ • Elbow: +100mm                                                    │   │
│  │ • Wrist: +50mm                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ Electronics Enclosure (30mm height)                                │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ Base Platform (20mm thickness)                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. ROBOTIC ARM ASSEMBLY

### 3.1 Arm Specifications

```
ROBOTIC ARM SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│                           5-DOF ROBOTIC ARM                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│ Degrees of Freedom: 5 (Base rotation + 4 joints)                           │
│                                                                             │
│ Joint Specifications:                                                       │
│ ┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐    │
│ │ Joint       │ Servo       │ Range       │ Torque      │ Weight      │    │
│ ├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤    │
│ │ Base        │ MG996R      │ 0-180°      │ 11kg·cm     │ 55g         │    │
│ │ Shoulder    │ MG996R      │ 0-180°      │ 11kg·cm     │ 55g         │    │
│ │ Elbow       │ MG996R      │ 0-180°      │ 11kg·cm     │ 55g         │    │
│ │ Wrist       │ MG996R      │ 0-180°      │ 11kg·cm     │ 55g         │    │
│ │ Gripper     │ SG90        │ 0-90°       │ 1.8kg·cm    │ 9g          │    │
│ └─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘    │
│                                                                             │
│ Reach Envelope: 300mm radius                                               │
│ Payload Capacity: 500g (with counterbalancing)                             │
│ Positioning Accuracy: ±2mm                                                 │
│ Total Weight: 1.2kg (excluding servos)                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Individual Segment Details

#### Base Segment
```
BASE SEGMENT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: Ø80mm × 50mm height                                            │
│                                                                             │
│ Features:                                                                  │
│ • Servo MG996R mounting (25mm × 50mm × 40mm cavity)                        │
│ • Bearing mount for smooth rotation (Ø22mm bearing)                        │
│ • Cable routing channels                                                   │
│ • Shoulder segment mounting interface                                      │
│                                                                             │
│ Mounting: 4× M3 screws to base platform                                    │
│ Weight: 120g                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Shoulder Segment
```
SHOULDER SEGMENT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 60mm × 40mm × 100mm                                            │
│                                                                             │
│ Features:                                                                  │
│ • Servo MG996R mounting with horn attachment                               │
│ • Elbow segment pivot point (Ø10mm shaft)                                  │
│ • Counterweight cavity for balance                                         │
│ • Cable management clips                                                   │
│                                                                             │
│ Range: 0-180° vertical movement                                            │
│ Weight: 85g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Elbow Segment
```
ELBOW SEGMENT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 50mm × 35mm × 100mm                                            │
│                                                                             │
│ Features:                                                                  │
│ • Servo MG996R mounting with linkage                                       │
│ • Wrist segment pivot point                                                │
│ • Lightweight design with internal ribbing                                 │
│ • Servo cable routing                                                       │
│                                                                             │
│ Range: 0-180° extension/retraction                                         │
│ Weight: 70g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Wrist Segment
```
WRIST SEGMENT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 40mm × 30mm × 50mm                                             │
│                                                                             │
│ Features:                                                                  │
│ • Servo MG996R mounting for rotation                                       │
│ • Gripper attachment interface                                             │
│ • Compact design for precision                                             │
│ • Cable strain relief                                                      │
│                                                                             │
│ Range: 0-180° rotation                                                     │
│ Weight: 45g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Gripper Assembly
```
GRIPPER ASSEMBLY SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 60mm × 40mm × 30mm (open)                                      │
│                                                                             │
│ Features:                                                                  │
│ • SG90 servo mounting                                                      │
│ • Dual-finger design with gear mechanism                                   │
│ • Rubber grip pads for material handling                                   │
│ • Opening range: 0-50mm                                                    │
│                                                                             │
│ Grip Force: 1.5kg (with gear advantage)                                    │
│ Weight: 35g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. ELECTRONICS ENCLOSURE

### 4.1 Raspberry Pi Mount
```
RASPBERRY PI 4B MOUNT:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 90mm × 60mm × 25mm                                             │
│                                                                             │
│ Features:                                                                  │
│ • Exact fit for Raspberry Pi 4B (85mm × 56mm)                              │
│ • GPIO header access cutout                                                │
│ • USB port access (2× USB 3.0, 1× USB-C)                                   │
│ • HDMI and audio jack access                                               │
│ • Ethernet port access                                                     │
│ • CSI camera connector access                                              │
│ • Heat sink mounting points                                                │
│ • Ventilation slots for cooling                                            │
│                                                                             │
│ Mounting: 4× M2.5 screws with standoffs                                    │
│ Material: ABS for heat resistance                                          │
│ Weight: 45g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Arduino Mega Mount
```
ARDUINO MEGA 2560 MOUNT:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 110mm × 65mm × 20mm                                            │
│                                                                             │
│ Features:                                                                  │
│ • Exact fit for Arduino Mega (101.6mm × 53.3mm)                            │
│ • Pin header access cutouts                                                │
│ • USB-B connector access                                                    │
│ • Power jack access                                                        │
│ • ICSP header access for programming                                       │
│ • Component clearance (30mm above board)                                   │
│ • Cable management slots                                                   │
│                                                                             │
│ Mounting: 4× M3 screws with nylon standoffs                                │
│ Material: PLA (heat resistant)                                             │
│ Weight: 55g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.3 Power Supply Housing
```
POWER SUPPLY HOUSING:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 85mm × 55mm × 35mm                                             │
│                                                                             │
│ Features:                                                                  │
│ • Fits standard 12V 3A power adapter                                       │
│ • Ventilation for heat dissipation                                         │
│ • Cable strain relief                                                      │
│ • Fuse holder mounting                                                     │
│ • LED indicator window                                                     │
│ • Terminal block access for distribution                                   │
│                                                                             │
│ Mounting: 4× M3 screws                                                     │
│ Material: ABS fire retardant                                               │
│ Weight: 40g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. SENSOR MOUNTS

### 5.1 Camera Mount
```
USB CAMERA MOUNT:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 50mm × 40mm × 60mm                                             │
│                                                                             │
│ Features:                                                                  │
│ • Adjustable angle mount (pan: ±45°, tilt: -15° to +45°)                   │
│ • Standard camera screw mount (1/4"-20)                                    │
│ • Cable management clips                                                   │
│ • Vibration dampening mounts                                               │
│ • Quick release mechanism                                                  │
│                                                                             │
│ Mounting: M5 ball joint for adjustability                                  │
│ Material: PETG for flexibility                                             │
│ Weight: 25g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 LIDAR Mount
```
LD06 LIDAR MOUNT:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: Ø100mm × 30mm height                                           │
│                                                                             │
│ Features:                                                                  │
│ • 360° clear scanning area                                                 │
│ • Adjustable height (20-50mm range)                                        │
│ • Cable routing for power and data                                         │
│ • Anti-vibration mounting                                                  │
│ • Calibration marks for alignment                                          │
│                                                                             │
│ Mounting: M4 screws with locknuts                                          │
│ Material: ABS for dimensional stability                                    │
│ Weight: 35g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.3 Ultrasonic Sensor Mount
```
ULTRASONIC SENSOR MOUNT:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 45mm × 25mm × 40mm                                             │
│                                                                             │
│ Features:                                                                  │
│ • Angled mounting for optimal detection                                    │
│ • Sensor protection cage                                                   │
│ • Cable strain relief                                                      │
│ • Adjustable positioning                                                   │
│                                                                             │
│ Mounting: M3 screws                                                        │
│ Material: PLA                                                              │
│ Weight: 15g                                                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. WASTE SORTING BINS

### 6.1 Bin Specifications
```
WASTE SORTING BIN SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Dimensions: 100mm × 100mm × 80mm (each)                                    │
│                                                                             │
│ Features:                                                                  │
│ • Tapered design for easy material insertion                               │
│ • Label areas for material type identification                             │
│ • Stackable design for storage                                             │
│ • Drainage holes in base                                                   │
│ • Reinforced corners for durability                                        │
│                                                                             │
│ Capacity: 2L per bin                                                       │
│ Material: PETG for chemical resistance                                     │
│ Weight: 65g each                                                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. MATERIAL SPECIFICATIONS

### 7.1 Printing Materials
```
RECOMMENDED PRINTING MATERIALS:
┌─────────────────┬─────────────┬─────────────┬─────────────┬─────────────┐
│ Component       │ Material    │ Temperature │ Infill      │ Purpose     │
├─────────────────┼─────────────┼─────────────┼─────────────┼─────────────┤
│ Base Platform   │ PETG        │ 250°C       │ 40%         │ Strength    │
│ Robotic Arm     │ ABS         │ 255°C       │ 30%         │ Durability  │
│ Electronics     │ ABS         │ 255°C       │ 25%         │ Heat resist │
│ Sensor Mounts   │ PLA+        │ 215°C       │ 20%         │ Precision   │
│ Waste Bins      │ PETG        │ 250°C       │ 15%         │ Chemical    │
└─────────────────┴─────────────┴─────────────┴─────────────┴─────────────┘
```

### 7.2 Hardware Materials
- **Fasteners:** Stainless steel M3/M4/M5 screws
- **Inserts:** Brass heat-set threaded inserts
- **Bearings:** 608ZZ ball bearings (8mm × 22mm × 7mm)
- **Gears:** POM plastic for wear resistance
- **Cables:** Flexible silicone-insulated wires

---

## 8. ASSEMBLY INSTRUCTIONS

### 8.1 Required Tools
- **3D Printer:** FDM printer with 200mm³ build volume
- **Screwdrivers:** Phillips #1, #2 and flathead 3mm
- **Allen Keys:** 2mm, 2.5mm, 3mm
- **Pliers:** Needle-nose for cable management
- **Wire Cutters/Strippers:** For cable preparation
- **Multimeter:** For electrical testing
- **Level:** For platform alignment

### 8.2 Assembly Sequence
1. **Base Platform Assembly**
   - Install threaded inserts
   - Mount electronics enclosures
   - Attach leveling feet

2. **Robotic Arm Assembly**
   - Install servos in each segment
   - Connect segments with bearings
   - Attach gripper assembly

3. **Sensor Installation**
   - Mount camera with adjustable bracket
   - Install LIDAR on dedicated mount
   - Position ultrasonic sensors

4. **Electronics Integration**
   - Install Raspberry Pi and Arduino
   - Connect power supply
   - Route and secure cables

5. **System Testing**
   - Power-on verification
   - Servo calibration
   - Sensor functionality testing

---

## 9. QUALITY CONTROL

### 9.1 Dimensional Checks
- **Critical Dimensions:** ±0.05mm tolerance
- **Hole Sizes:** Standard metric clearances
- **Alignment:** ±0.1mm for mating features
- **Flatness:** 0.2mm maximum deviation

### 9.2 Functional Testing
- **Fit Testing:** All parts assemble without force
- **Clearance Checks:** Moving parts have proper clearance
- **Load Testing:** Robotic arm can lift specified payload
- **Stability:** System remains stable during operation

---

## 10. MAINTENANCE AND UPDATES

### 10.1 Regular Maintenance
- **Cleaning:** Remove dust and debris weekly
- **Lubrication:** Apply lubricant to moving parts monthly
- **Calibration:** Recalibrate servos quarterly
- **Inspection:** Check for wear and damage monthly

### 10.2 Version Control
- **File Naming:** Component_v1.0.STL
- **Backup:** Maintain previous versions
- **Change Log:** Document all modifications
- **Compatibility:** Ensure backward compatibility

---

## 11. REFERENCE DOCUMENTS

- ASME Y14.5-2018 Dimensioning and Tolerancing
- ISO 1101:2017 Geometrical Product Specifications
- ASTM D638 Plastic Material Testing Standards
- IEEE 315-1975 Electrical Diagram Symbols

---

## 12. DOCUMENT REVISION HISTORY

| Revision | Date | Author | Changes |
|----------|------|--------|---------|
| 1.0 | 2025-11-17 | RoboSort Team | Initial release with complete 3D model specifications |

---

**Note:** All 3D models are designed for FDM printing with standard settings. Test prints are recommended before full production runs. Dimensions are in millimeters unless otherwise specified.
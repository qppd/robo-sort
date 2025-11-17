# RoboSort Visual Diagram Specifications

## Professional Diagram Standards

**Document Information:**
- **Document Title:** RoboSort Visual Diagram Specifications
- **Document Number:** RBS-VIS-001
- **Revision:** 1.0
- **Date:** November 17, 2025
- **Author:** RoboSort Development Team
- **Standards:** ISO 128-1:2020, IEEE 315-1975

---

## 1. WIRING DIAGRAM IMAGE SPECIFICATIONS

### 1.1 Overall Layout
```
WIRING DIAGRAM LAYOUT SPECIFICATION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Title Block: RoboSort System Wiring Diagram                                │
│ Revision: 1.0 | Date: 2025-11-17 | Scale: NTS                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      POWER DISTRIBUTION                             │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │   │
│  │  │ 12V Supply  │───►│   Fuses     │───►│ Regulators  │             │   │
│  │  │  (Input)    │    │  (F1,F2)    │    │  (5V,12V)   │             │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    MICROCONTROLLER SECTION                          │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │   │
│  │  │ Arduino     │    │ Raspberry   │    │   USB       │             │   │
│  │  │ Mega 2560   │    │    Pi 4B    │    │  Cable      │             │   │
│  │  │             │    │             │    │             │             │   │
│  │  │ • PWM Pins  │    │ • USB Ports │    │ • Data      │             │   │
│  │  │ • Digital I/O│    │ • CSI Cam  │    │ • Power     │             │   │
│  │  │ • Serial    │    │ • HDMI      │    │             │             │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       ACTUATOR SECTION                             │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │   │
│  │  │ Servo       │    │ Stepper     │    │ PWM Signals │             │   │
│  │  │ Motors      │    │ Driver      │    │ (3-7)       │             │   │
│  │  │ (5× MG996R) │    │ (A4988)     │    │             │             │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       SENSOR SECTION                               │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │   │
│  │  │ USB Camera  │    │ LD06 LIDAR  │    │ HC-SR04     │             │   │
│  │  │ (Vision)    │    │ (360°)      │    │ Ultrasonic  │             │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│ Notes:                                                                    │
│ • All connections shown with standard electrical symbols                 │
│ • Wire colors indicated by legend                                         │
│ • Pin numbers clearly labeled                                             │
│ • Safety grounds highlighted                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Symbol Legend
```
ELECTRICAL SYMBOL LEGEND:
┌─────────────────────────────────────────────────────────────────────────────┐
│                           STANDARD SYMBOLS                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ Power Symbols:                                                             │
│ ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    │
│ │   +12V DC   │    │    +5V DC   │    │   Ground    │    │   Fuse      │    │
│ │  Supply     │    │  Supply     │    │  (GND)     │    │             │    │
│ └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    │
│                                                                             │
│ Connection Symbols:                                                        │
│ ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    │
│ │ Wire Cross  │    │  Junction   │    │ Connector   │    │  Terminal   │    │
│ │             │    │             │    │             │    │   Block     │    │
│ └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    │
│                                                                             │
│ Component Symbols:                                                         │
│ ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    │
│ │ Resistor    │    │ Capacitor   │    │  Switch     │    │   LED       │    │
│ │             │    │             │    │             │    │             │    │
│ └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    │
│                                                                             │
│ Signal Types:                                                              │
│ ──────────────  Digital Signal                                             │
│ ~~~~~~~~~~~~~  Analog Signal                                               │
│ ─ ─ ─ ─ ─ ─ ─  PWM Signal                                                  │
│ =============  Power Rail                                                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Color Coding Standard
```
WIRE COLOR CODING STANDARD:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Function          │ Primary Color │ Secondary │ Usage                      │
├───────────────────┼───────────────┼───────────┼────────────────────────────┤
│ Power (+12V)      │ Red           │ White     │ Main power distribution    │
│ Power (+5V)       │ Red           │ Yellow    │ Logic power                │
│ Ground (GND)      │ Black         │ -         │ Common reference           │
│ Digital Signals   │ Blue          │ White     │ Control signals           │
│ PWM Signals       │ Purple        │ White     │ Servo control             │
│ Serial TX         │ Green         │ White     │ Data transmission         │
│ Serial RX         │ White         │ Green     │ Data reception            │
│ Analog Signals    │ Orange        │ White     │ Sensor inputs             │
│ USB Data+         │ Green         │ -         │ USB differential          │
│ USB Data-         │ White         │ -         │ USB differential          │
│ Stepper A+        │ Yellow        │ -         │ Motor coil A              │
│ Stepper A-        │ Yellow        │ Black     │ Motor coil A              │
│ Stepper B+        │ Blue          │ -         │ Motor coil B              │
│ Stepper B-        │ Blue          │ Black     │ Motor coil B              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.4 Detailed Component Diagrams

#### Arduino Mega Pin Detail
```
ARDUINO MEGA PIN DETAIL (Visual Layout):
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ARDUINO MEGA 2560                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  POWER HEADER (Left Side):                                                  │
│  ┌───┬───┬───┬───┬───┬───┬───┬───┐                                         │
│  │IO │VIN│GND│GND│5V │3V3│RST│IO │  ← Pins                                │
│  │REF│   │   │   │   │   │   │REF│  ← Functions                            │
│  └───┴───┴───┴───┴───┴───┴───┴───┘                                         │
│                                                                             │
│  ANALOG HEADER (Right Side):                                                │
│  ┌───┬───┬───┬───┬───┬───┬───┬───┐                                         │
│  │A0 │A1 │A2 │A3 │A4 │A5 │A6 │A7 │  ← Pins                                │
│  │   │   │   │   │SDA│SCL│   │   │  ← Functions                            │
│  └───┴───┴───┴───┴───┴───┴───┴───┘                                         │
│                                                                             │
│  DIGITAL HEADERS (Top and Bottom):                                          │
│  Top:    0  1  2  3  4  5  6  7  8  9 10 11 12 13                       │
│  Bottom: 14 15 16 17 18 19 20 21 22 23 24 25 26 27                       │
│                                                                             │
│  SPECIAL CONNECTIONS:                                                       │
│  • USB-B Connector (Programming)                                           │
│  • DC Power Jack (7-12V)                                                   │
│  • ICSP Headers (ISP Programming)                                          │
│  • Reset Button                                                            │
│                                                                             │
│  DIMENSIONS: 101.6mm × 53.3mm                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Stepper Driver Detail
```
A4988 STEPPER DRIVER DETAIL:
┌─────────────────────────────────────────────────────────────────────────────┐
│                            A4988 DRIVER MODULE                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  PIN LAYOUT:                                                               │
│  ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐        │
│  │EN │MS1│MS2│MS3│RST│SLP│STP│DIR│GND│GND│2B │2A │1A │1B │VM │VDD│        │
│  └───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┘        │
│                                                                             │
│  CONNECTION DETAILS:                                                       │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐                 │
│  │ Pin         │ Arduino Pin │ Function    │ Notes       │                 │
│  ├─────────────┼─────────────┼─────────────┼─────────────┤                 │
│  │ ENABLE      │ 10          │ Enable/Disable│ Active Low │                 │
│  │ MS1         │ 11          │ Microstep 1 │ Step size   │                 │
│  │ MS2         │ 12          │ Microstep 2 │ Step size   │                 │
│  │ MS3         │ 13          │ Microstep 3 │ Step size   │                 │
│  │ STEP        │ 8           │ Step Pulse  │ High freq   │                 │
│  │ DIR         │ 9           │ Direction   │ 0=CW,1=CCW │                 │
│  │ VMOT        │ 12V Supply  │ Motor Power │ 8-35V       │                 │
│  │ VDD         │ 5V Supply   │ Logic Power │ 3-5.5V      │                 │
│  │ GND         │ Ground      │ Common      │ -           │                 │
│  └─────────────┴─────────────┴─────────────┴─────────────┘                 │
│                                                                             │
│  MOTOR COIL CONNECTIONS:                                                    │
│  ┌─────────────┬─────────────┬─────────────┐                               │
│  │ Coil        │ Driver Pin  │ Wire Color │                               │
│  ├─────────────┼─────────────┼─────────────┤                               │
│  │ A+          │ 1A          │ Yellow     │                               │
│  │ A-          │ 1B          │ Yellow/Blk │                               │
│  │ B+          │ 2A          │ Blue       │                               │
│  │ B-          │ 2B          │ Blue/Blk   │                               │
│  └─────────────┴─────────────┴─────────────┘                               │
│                                                                             │
│  DIMENSIONS: 20mm × 15mm                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 3D MODEL IMAGE SPECIFICATIONS

### 2.1 Rendering Standards
```
3D MODEL RENDERING REQUIREMENTS:
┌─────────────────────────────────────────────────────────────────────────────┐
│                           RENDERING STANDARDS                               │
├─────────────────────────────────────────────────────────────────────────────┤
│ Software: Blender 3.6+, Fusion 360, KeyShot                                │
│                                                                             │
│ Lighting:                                                                  │
│ • Three-point lighting setup                                               │
│ • Key light: 45° from camera                                               │
│ • Fill light: Opposite side, 50% intensity                                 │
│ • Rim light: Behind subject, 25% intensity                                 │
│                                                                             │
│ Materials:                                                                 │
│ • Plastic: PBR shader with subsurface scattering                           │
│ • Metal: Reflective with proper IOR values                                 │
│ • Glass: Transparent with refraction                                        │
│ • Rubber: Soft, matte finish                                               │
│                                                                             │
│ Camera Settings:                                                           │
│ • Lens: 50mm equivalent                                                    │
│ • Aperture: f/8 for depth of field                                          │
│ • ISO: 100                                                                 │
│ • Resolution: 1920×1080 minimum                                            │
│                                                                             │
│ Post-Processing:                                                           │
│ • Color correction for consistency                                         │
│ • Slight vignette for focus                                                │
│ • Sharpening for detail enhancement                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 View Specifications

#### Isometric View
```
ISOMETRIC VIEW SPECIFICATION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Camera Position: 30° elevation, 45° azimuth                                │
│                                                                             │
│ Content:                                                                   │
│ • Complete assembly with all components                                    │
│ • Transparent materials for internal visibility                            │
│ • Dimension callouts for key measurements                                  │
│ • Exploded view annotations for assembly                                   │
│                                                                             │
│ Annotations:                                                               │
│ • Component labels with leader lines                                       │
│ • Material callouts                                                        │
│ • Assembly sequence numbers                                                │
│ • Safety warnings                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Exploded View
```
EXPLODED VIEW SPECIFICATION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Layout: Components separated radially from center                          │
│                                                                             │
│ Separation Distance: 50-100mm between components                           │
│                                                                             │
│ Leader Lines: Curved arrows showing assembly direction                     │
│                                                                             │
│ Callouts:                                                                  │
│ • Part numbers                                                             │
│ • Quantity required                                                        │
│ • Assembly sequence                                                        │
│ • Special tools required                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Cross-Section View
```
CROSS-SECTION VIEW SPECIFICATION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Cutting Plane: Through center of robotic arm                               │
│                                                                             │
│ Display:                                                                   │
│ • Internal mechanisms visible                                              │
│ • Hidden lines removed                                                     │
│ • Section lines at 45° angle                                               │
│ • Different patterns for different materials                               │
│                                                                             │
│ Annotations:                                                               │
│ • Internal dimensions                                                     │
│ • Clearance specifications                                                 │
│ • Tolerance callouts                                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.3 Component Detail Views

#### Robotic Arm Detail
```
ROBOTIC ARM DETAIL VIEW:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Focus: Individual joint mechanisms                                        │
│                                                                             │
│ Details to Show:                                                           │
│ • Servo motor mounting                                                     │
│ • Gear reduction systems                                                   │
│ • Bearing locations                                                        │
│ • Cable routing paths                                                      │
│ • Fastener locations                                                       │
│                                                                             │
│ Close-up Areas:                                                            │
│ • Joint clearances                                                         │
│ • Bearing fits                                                             │
│ • Servo horn attachments                                                   │
│ • Gripper mechanism                                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Electronics Detail
```
ELECTRONICS DETAIL VIEW:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Focus: PCB layouts and component placement                                 │
│                                                                             │
│ Details to Show:                                                           │
│ • Component footprints                                                     │
│ • Pin connections                                                          │
│ • Heat sink placement                                                      │
│ • Cable routing                                                            │
│ • Ventilation paths                                                        │
│                                                                             │
│ Technical Details:                                                         │
│ • Trace widths                                                             │
│ • Via sizes                                                                │
│ • Component spacing                                                        │
│ • EMI shielding                                                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.4 Animation Specifications
```
ANIMATION SEQUENCE SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Sequence 1: Assembly Animation                                             │
│ • Duration: 30 seconds                                                    │
│ • Components appear in assembly order                                      │
│ • Fasteners animate into place                                             │
│ • Cables route automatically                                               │
│                                                                             │
│ Sequence 2: Operation Animation                                           │
│ • Duration: 45 seconds                                                    │
│ • Robotic arm movement cycle                                               │
│ • Servo rotation visualization                                             │
│ • Gripper open/close sequence                                              │
│                                                                             │
│ Sequence 3: Maintenance Animation                                         │
│ • Duration: 20 seconds                                                    │
│ • Disassembly sequence                                                     │
│ • Component removal visualization                                          │
│ • Access panel opening                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. BILL OF MATERIALS VISUALIZATION

### 3.1 BOM Table Format
```
BILL OF MATERIALS TABLE SPECIFICATION:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Item │ Part Number │ Description          │ Qty │ Material    │ Supplier   │
├──────┼─────────────┼──────────────────────┼─────┼─────────────┼────────────┤
│ 1    │ RBS-BASE-01 │ Main Base Platform   │ 1   │ PETG        │ Printed    │
│ 2    │ MG996R-01   │ Servo Motor          │ 5   │ -           │ TowerPro   │
│ 3    │ A4988-01    │ Stepper Driver       │ 1   │ -           │ Pololu     │
│ ...  │ ...         │ ...                  │ ... │ ...         │ ...        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Component Callouts
```
COMPONENT CALLOUT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ • Circular labels with numbers                                             │
│ • Leader lines connecting to components                                    │
│ • Consistent numbering system (1, 2, 3...)                                 │
│ • Font: Arial Bold, 12pt minimum                                          │
│ • Color: White background, black text                                      │
│ • Position: Clear of other elements                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. TECHNICAL ILLUSTRATION STANDARDS

### 4.1 Line Weights
```
LINE WEIGHT SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Line Type          │ Weight │ Color      │ Usage                          │
├────────────────────┼────────┼────────────┼────────────────────────────────┤
│ Object Outline     │ 2pt    │ Black      │ Main component boundaries      │
│ Hidden Lines       │ 1pt    │ Gray       │ Internal features              │
│ Center Lines       │ 0.5pt  │ Blue       │ Symmetry and centers           │
│ Dimension Lines    │ 1pt    │ Black      │ Measurement indicators         │
│ Extension Lines    │ 0.5pt  │ Black      │ Dimension line extensions      │
│ Leader Lines       │ 1pt    │ Black      │ Annotation pointers            │
│ Section Lines      │ 1pt    │ Red        │ Cut plane indicators           │
└─────────────────────────────────────────────────────────────────────────────┴────────────────────────────────┘
```

### 4.2 Dimensioning
```
DIMENSIONING STANDARDS (ASME Y14.5):
┌─────────────────────────────────────────────────────────────────────────────┐
│ • Unidirectional dimensioning                                             │
│ • Decimal inches or millimeters                                           │
│ • Tolerance: ±0.1mm for general, ±0.05mm for critical                     │
│ • Font: Arial, 10pt minimum                                               │
│ • Arrow style: Closed filled arrows                                        │
│ • Extension line gap: 2mm minimum                                         │
│ • Text placement: Above dimension line                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.3 Scaling and Proportions
```
SCALING SPECIFICATIONS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ • Main assembly views: 1:10 or 1:5 scale                                   │
│ • Detail views: 2:1 or 1:1 scale                                          │
│ • Consistent scaling across all views                                      │
│ • Scale indicator in title block                                           │
│ • Proportional representation of all components                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. FILE FORMAT SPECIFICATIONS

### 5.1 Image Formats
```
RECOMMENDED IMAGE FORMATS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Format │ Resolution │ DPI   │ Usage                     │ Compression     │
├────────┼────────────┼───────┼───────────────────────────┼─────────────────┤
│ PNG    │ 1920×1080  │ 300   │ Technical illustrations   │ Lossless        │
│ JPEG   │ 1920×1080  │ 300   │ Photographs              │ 90% quality     │
│ SVG    │ Vector     │ N/A   │ Diagrams and schematics  │ Vector format   │
│ TIFF   │ 1920×1080  │ 300   │ Archive and printing     │ Lossless        │
│ PDF    │ Vector     │ N/A   │ Multi-page documents     │ Vector format   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 3D Model Formats
```
3D MODEL EXPORT FORMATS:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Format │ Extension │ Usage                     │ Software Support         │
├────────┼───────────┼───────────────────────────┼──────────────────────────┤
│ STL    │ .stl      │ 3D Printing               │ All slicers              │
│ OBJ    │ .obj      │ 3D Rendering              │ Most 3D software         │
│ FBX    │ .fbx      │ Game engines              │ Unity, Unreal            │
│ GLTF   │ .gltf     │ Web display               │ Browsers, AR apps        │
│ STEP   │ .step     │ CAD exchange              │ Professional CAD         │
│ IGES   │ .iges     │ CAD exchange              │ Legacy CAD systems       │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. QUALITY ASSURANCE

### 6.1 Review Checklist
```
VISUAL DIAGRAM REVIEW CHECKLIST:
□ All components labeled clearly
□ Dimensions accurate and complete
□ Line weights consistent
□ Colors follow standards
□ Text legible (minimum 10pt)
□ No overlapping elements
□ Professional appearance
□ Scale indicated
□ Revision information current
□ Copyright notice included
□ File naming convention followed
```

### 6.2 Approval Process
```
APPROVAL WORKFLOW:
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Draft     │───►│   Review    │───►│   Revise    │───►│   Approve   │
│             │    │             │    │             │    │             │
│ • Initial   │    │ • Technical │    │ • Address   │    │ • Final     │
│ • Layout    │    │ • Accuracy  │    │ • Comments  │    │ • Release   │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

---

## 7. DOCUMENT REVISION HISTORY

| Revision | Date | Author | Changes |
|----------|------|--------|---------|
| 1.0 | 2025-11-17 | RoboSort Team | Initial release with complete visual specifications |

---

**Note:** These specifications ensure professional, consistent, and technically accurate visual documentation for the RoboSort system. All diagrams should be created using industry-standard software and follow the specified conventions for maximum clarity and usability.
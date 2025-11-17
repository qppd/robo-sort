# RoboSort System Diagrams

This directory contains comprehensive visual documentation of the RoboSort system architecture, workflows, and configurations.

## Diagram Files

### 1. `system_architecture.puml`
**Format**: PlantUML
**Description**: Complete system architecture diagram showing all components and their interactions
**Usage**:
- View online at: http://www.plantuml.com/plantuml/
- Generate PNG/SVG using PlantUML tools
- Shows data flow between Raspberry Pi, Arduino, and hardware components

### 2. `workflow_diagrams.md`
**Format**: Mermaid (GitHub-compatible)
**Description**: Interactive workflow diagrams showing processing workflows, communication sequences, and state transitions
**Contents**:
- Main processing loop flowchart
- Serial communication sequence diagram
- System state transitions
**Usage**: View directly on GitHub or any Markdown renderer that supports Mermaid

### 3. `pin_configuration.txt`
**Format**: ASCII Text Diagram
**Description**: Detailed hardware pin configuration and wiring guide
**Contents**:
- Arduino Mega pinout with component connections
- Power distribution schematic
- Connection verification checklist
- Troubleshooting guide
**Usage**: Reference for hardware assembly and debugging

### 4. `wiring_diagram_specification.md`
**Format**: Professional Documentation
**Description**: Complete wiring diagram specifications following industry standards
**Contents**:
- Detailed component pinouts
- Wire gauge requirements
- Color coding standards
- Safety specifications
- Testing procedures
**Usage**: Technical reference for creating professional wiring diagrams

### 5. `3d_model_specification.md`
**Format**: Professional Documentation
**Description**: Comprehensive 3D model specifications and assembly details
**Contents**:
- Component dimensions and materials
- Assembly instructions
- Quality control procedures
- Maintenance guidelines
**Usage**: Technical reference for 3D modeling and manufacturing

### 6. `visual_diagram_specifications.md`
**Format**: Professional Standards
**Description**: Visual diagram creation standards and specifications
**Contents**:
- Wiring diagram layout standards
- 3D rendering requirements
- Technical illustration standards
- File format specifications
**Usage**: Guidelines for creating professional visual documentation

## System Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Raspberry Pi  │    │   Arduino Mega  │    │   Hardware      │
│   (High-Level)  │◄──►│   (Low-Level)   │◄──►│   Components    │
│                 │    │                 │    │                 │
│ • AI Vision     │    │ • Servo Control │    │ • Robotic Arm   │
│ • YOLO Detect   │    │ • Stepper Motor │    │ • Servos        │
│ • LIDAR Process │    │ • Sensor Read   │    │ • Stepper Motor │
│ • Decision      │    │ • Serial Comm   │    │ • LIDAR Sensor  │
│ • Coordination  │    │                 │    │ • Ultrasonic    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │
         └────────────────────────┼────────────────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │     Serial USB (9600)     │
                    │   Bidirectional Protocol  │
                    └───────────────────────────┘
```

## Professional Documentation Standards

### IEEE Standards Compliance
- **IEEE 315-1975**: Graphic Symbols for Electrical Diagrams
- **IEEE 91-1984**: Logic Symbols
- **IEEE 315A-1986**: Supplement to IEEE 315-1975

### ASME Standards Compliance
- **ASME Y14.5-2018**: Dimensioning and Tolerancing
- **ASME Y14.3-2012**: Orthographic and Pictorial Views

### ISO Standards Compliance
- **ISO 128-1:2020**: Technical Product Documentation - General Principles
- **ISO 1101:2017**: Geometrical Product Specifications

## Key Visualizations

### System Component Relationships
- **Data Flow**: Camera → AI → Decision → Hardware control
- **Communication Protocols**: Serial USB communication at 9600bps
- **Power Distribution**: Centralized 12V supply with regulated outputs
- **Control Signals**: PWM for servos, STEP/DIR for stepper motors

### Hardware Specifications
- **Dimensions**: Complete measurements for all components
- **Materials**: Recommended printing materials and specifications
- **Tolerances**: Precision requirements for proper fit and function
- **Assembly**: Step-by-step assembly procedures

### Electrical Specifications
- **Pin Configurations**: Detailed Arduino Mega pin assignments
- **Wire Gauges**: Appropriate wire sizes for different applications
- **Color Coding**: Industry-standard wire color conventions
- **Safety**: Protection circuits and safety considerations

## Usage Guidelines

### For Hardware Assembly
1. **Reference `pin_configuration.txt`** for Arduino connections
2. **Use `wiring_diagram_specification.md`** for electrical standards
3. **Follow `3d_model_specification.md`** for mechanical assembly
4. **Consult `visual_diagram_specifications.md`** for diagram creation

### For Documentation
1. **Use PlantUML** for system architecture diagrams
2. **Use Mermaid** for workflow and sequence diagrams
3. **Follow IEEE standards** for electrical schematics
4. **Follow ASME standards** for mechanical drawings

### For Manufacturing
1. **Reference 3D model specifications** for printing parameters
2. **Use BOM tables** for component procurement
3. **Follow quality control procedures** for consistency
4. **Maintain revision control** for design changes

## Tools for Viewing Diagrams

### Diagram Creation Tools
- **PlantUML**: Online viewer at plantuml.com or VS Code extension
- **Mermaid**: Built into GitHub, GitLab, and many Markdown viewers
- **Draw.io**: Free diagram creation tool
- **KiCad**: Professional electrical schematic software
- **Fusion 360**: Professional CAD software

### 3D Visualization Tools
- **Blender**: Free 3D creation and rendering
- **Fusion 360**: Professional CAD with rendering
- **KeyShot**: High-quality rendering software
- **Meshlab**: 3D model processing and analysis

## File Organization

```
diagrams/
├── README.md                           # This file
├── system_architecture.puml           # PlantUML architecture
├── workflow_diagrams.md               # Mermaid workflows
├── pin_configuration.txt              # ASCII pin diagrams
├── wiring_diagram_specification.md    # Professional wiring specs
├── 3d_model_specification.md          # 3D model documentation
└── visual_diagram_specifications.md   # Visual standards guide
```

## Contributing

When adding new diagrams:
1. **Follow established naming conventions**
2. **Include format information and usage instructions**
3. **Add descriptions of what the diagram shows**
4. **Update this README with new diagram information**
5. **Ensure compliance with professional standards**
6. **Include revision history and approval workflow**

## Quality Assurance

### Review Checklist
- [ ] Diagrams follow professional standards
- [ ] All components are accurately represented
- [ ] Dimensions and specifications are correct
- [ ] Color schemes are consistent
- [ ] Text is legible and properly sized
- [ ] File formats are appropriate for intended use
- [ ] Documentation is complete and current

### Approval Process
1. **Draft Creation**: Initial diagram development
2. **Technical Review**: Accuracy and completeness check
3. **Standards Compliance**: Verify against professional standards
4. **Peer Review**: Cross-check by team members
5. **Final Approval**: Release for documentation use

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Raspberry Pi  │    │   Arduino Mega  │    │   Hardware      │
│   (High-Level)  │◄──►│   (Low-Level)   │◄──►│   Components    │
│                 │    │                 │    │                 │
│ • AI Vision     │    │ • Servo Control │    │ • Robotic Arm   │
│ • YOLO Detect   │    │ • Stepper Motor │    │ • Servos        │
│ • LIDAR Process │    │ • Sensor Read   │    │ • Stepper Motor │
│ • Decision      │    │ • Serial Comm   │    │ • LIDAR Sensor  │
│ • Coordination  │    │                 │    │ • Ultrasonic    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │
         └────────────────────────┼────────────────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │     Serial USB (9600)     │
                    │   Bidirectional Protocol  │
                    └───────────────────────────┘
```

## Key Workflows

### AI Vision Processing Flow
1. Camera captures frame
2. YOLO model processes image
3. Object detection results generated
4. LIDAR data fused with vision data
5. Distance information overlaid on display

### Serial Communication Sequence
1. Raspberry Pi connects to Arduino via USB
2. Serial port established at 9600 baud
3. Arduino signals ready state
4. Commands sent from Pi to Arduino
5. Hardware actions executed
6. Response sent back to Pi

### Hardware Control Flow
1. Decision made by AI system
2. Command formatted for Arduino
3. Serial transmission to Arduino
4. Command parsing and validation
5. Hardware actuation (servos/stepper)
6. Status response returned

## Component Interactions

### Data Flow
- **Input**: Camera frames, LIDAR scans, sensor data
- **Processing**: AI inference, sensor fusion, decision making
- **Output**: Control commands, display updates, logging

### Control Flow
- **High-level**: Raspberry Pi manages AI and coordination
- **Low-level**: Arduino handles real-time hardware control
- **Communication**: Bidirectional serial protocol at 9600bps

### Power Flow
- **Main Supply**: 12V DC input to Arduino
- **Regulation**: 5V outputs for sensors and servos
- **Distribution**: Separate supplies for motors and logic

## Usage Guidelines

1. **Hardware Assembly**: Use `pin_configuration.txt` for wiring
2. **System Understanding**: Refer to PlantUML diagram for architecture
3. **Workflow Analysis**: Use Mermaid diagrams for process flows
4. **Troubleshooting**: Check pin connections and power distribution
5. **Development**: Understand component interactions before coding

## Tools for Viewing Diagrams

- **PlantUML**: Online viewer at plantuml.com or VS Code extension
- **Mermaid**: Built into GitHub, GitLab, and many Markdown viewers
- **ASCII Diagrams**: View in any text editor or terminal

## Contributing

When adding new diagrams:
1. Use consistent naming conventions
2. Include format information and usage instructions
3. Add descriptions of what the diagram shows
4. Update this README with new diagram information
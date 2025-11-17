# RoboSort System Diagrams

This directory contains various diagrams and visual representations of the RoboSort system architecture, workflows, and configurations.

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
**Description**: Interactive workflow diagrams showing system operation flow
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
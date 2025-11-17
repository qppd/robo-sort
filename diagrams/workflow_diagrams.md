```mermaid
graph TD
    A[Start System] --> B[Initialize Components]
    B --> C[Setup Camera]
    B --> D[Setup Serial Connection]
    B --> E[Setup LIDAR Sensor]

    C --> F[Camera Ready]
    D --> G[Serial Connected]
    E --> H[LIDAR Ready]

    F --> I{Main Processing Loop}
    G --> I
    H --> I

    I --> J[Capture Frame]
    J --> K[Run YOLO Detection]
    K --> L[Process Results]
    L --> M[Get LIDAR Data]
    M --> N[Fuse Sensor Data]
    N --> O{Material Detected?}

    O -->|Yes| P[Classify Material]
    O -->|No| Q[Continue Monitoring]

    P --> R{Classification}
    R -->|Paper| S[Send Paper Command]
    R -->|Plastic| T[Send Plastic Command]
    R -->|Other| U[Send Other Command]

    S --> V[Execute Robotic Action]
    T --> V
    U --> V

    V --> W[Display Results]
    W --> X[Log Data]
    X --> Y{Continue?}
    Y -->|Yes| I
    Y -->|No| Z[Shutdown System]

    Q --> I

    Z --> AA[End]
```

```mermaid
sequenceDiagram
    participant R as Raspberry Pi
    participant A as Arduino Mega
    participant H as Hardware

    R->>A: Connect USB Serial (9600bps)
    A-->>R: Ready Signal
    R->>R: Initialize Camera
    R->>R: Initialize LIDAR
    R->>R: Load YOLO Model

    loop Main Processing Loop
        R->>R: Capture Frame
        R->>R: Run YOLO Inference
        R->>R: Read LIDAR Data
        R->>R: Fuse Detection Results

        alt Object Detected
            R->>R: Classify Material
            R->>A: Send Control Command (e.g., "S0 90\n")
            A->>A: Parse Command
            A->>H: Execute Hardware Action
            H-->>A: Action Complete
            A-->>R: Response ("OK\n")
        end
    end

    R->>R: Display Results
    R->>R: Check Exit Condition
```

```mermaid
stateDiagram-v2
    [*] --> Initialization
    Initialization --> CameraSetup
    Initialization --> SerialSetup
    Initialization --> LIDARSetup

    CameraSetup --> Ready: Camera OK
    SerialSetup --> Ready: Serial OK
    LIDARSetup --> Ready: LIDAR OK

    Ready --> Processing: All Systems Ready

    state Processing as "Main Processing" as P
    P --> Detection: Capture Frame
    Detection --> Classification: Run YOLO
    Classification --> Fusion: Get LIDAR Data
    Fusion --> Decision: Analyze Results

    Decision --> Action: Object Found
    Decision --> Monitoring: No Object

    Action --> Command: Generate Command
    Command --> Execute: Send to Arduino
    Execute --> Response: Hardware Action
    Response --> Display: Update UI
    Display --> Processing

    Monitoring --> Processing

    Processing --> [*]: Exit Condition
```
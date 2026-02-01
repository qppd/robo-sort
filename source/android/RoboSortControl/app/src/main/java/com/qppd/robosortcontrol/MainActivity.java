package com.qppd.robosortcontrol;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.button.MaterialButton;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.io.BufferedInputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

public class MainActivity extends AppCompatActivity {

    // Firebase
    private FirebaseDatabase database;
    private DatabaseReference commandsRef;
    private DatabaseReference feedbackRef;
    
    // UI Elements
    private TextView statusText;
    private TextView feedbackText;
    private TextView servo1Label, servo2Label, servo3Label, servo4Label, servo5Label;
    
    private MaterialButton btnForward, btnBackward, btnLeft, btnRight, btnStop;
    private MaterialButton btnLifterUp, btnLifterDown, btnLifterStop;
    private MaterialButton btnBinHome, btnBin1, btnBin2, btnBin3, btnBin4;

    // Servo buttons (replace sliders)
    private MaterialButton btnArmRotateFront, btnArmRotateBack;
    private MaterialButton btnGripOpen, btnGripClose;
    private MaterialButton btnGripRotateMinus, btnGripRotatePlus;
    private MaterialButton btnGripRotateDownDiag, btnGripRotateUpDiag, btnGripRotateVertical, btnGripRotateHorizontal;
    private MaterialButton btnArmExtendMinus, btnArmExtendPlus;
    private MaterialButton btnArmExtend180, btnArmExtend110, btnArmExtend90;
    private MaterialButton btnLookMinus, btnLookPlus;
    
    // Object detection buttons
    private MaterialButton btnDetectPlasticBottle, btnDetectPlasticWrapper, btnDetectPaper, btnDetectOther;
    private MaterialButton btnDetectClear;
    
    // State
    private boolean isConnected = false;
    private Handler handler;
    
    // Servo positions
    // Match Arduino SERVO_CONFIG defaults (channels 1-5) + lifter buttons
    private int servo1Pos = 180;
    private int servo2Pos = 110;
    private int servo3Pos = 90;
    private int servo4Pos = 110;
    private int servo5Pos = 180;

    // Step sizes for +/- buttons
    private static final int STEP_GRIP_ROTATE = 10;
    private static final int STEP_ARM_EXTEND = 10;
    private static final int STEP_LOOK = 10;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        handler = new Handler(Looper.getMainLooper());
        
        // Initialize Firebase
        initializeFirebase();
        
        // Initialize UI
        initializeUI();
        
        // Setup listeners
        setupMotorControls();
        setupLifterControls();
        setupBinControls();
        setupServoControls();
        setupObjectDetectionControls();
        setupFeedbackListener();
    }
    
    private void initializeFirebase() {
        try {
            database = FirebaseDatabase.getInstance();
            // REMOVED: database.useEmulator("10.0.2.2", 9000); // For production use
            
            commandsRef = database.getReference("robosort/commands");
            // RPi publishes live state under robosort/status
            feedbackRef = database.getReference("robosort/status");
            
            // Test connection with detailed logging
            commandsRef.child("timestamp").setValue(System.currentTimeMillis())
                .addOnSuccessListener(aVoid -> {
                    isConnected = true;
                    updateConnectionStatus(true);
                    feedbackText.setText("Firebase connected successfully!");
                })
                .addOnFailureListener(e -> {
                    isConnected = false;
                    updateConnectionStatus(false);
                    feedbackText.setText("Firebase connection failed: " + e.getMessage());
                    e.printStackTrace();
                });
                
        } catch (Exception e) {
            e.printStackTrace();
            updateConnectionStatus(false);
            feedbackText.setText("Firebase initialization error: " + e.getMessage());
        }
    }
    
    private void initializeUI() {
        statusText = findViewById(R.id.statusText);
        feedbackText = findViewById(R.id.feedbackText);
        
        btnForward = findViewById(R.id.btnForward);
        btnBackward = findViewById(R.id.btnBackward);
        btnLeft = findViewById(R.id.btnLeft);
        btnRight = findViewById(R.id.btnRight);
        btnStop = findViewById(R.id.btnStop);

        btnLifterUp = findViewById(R.id.btnLifterUp);
        btnLifterDown = findViewById(R.id.btnLifterDown);
        btnLifterStop = findViewById(R.id.btnLifterStop);
        
        btnBinHome = findViewById(R.id.btnBinHome);
        btnBin1 = findViewById(R.id.btnBin1);
        btnBin2 = findViewById(R.id.btnBin2);
        btnBin3 = findViewById(R.id.btnBin3);
        btnBin4 = findViewById(R.id.btnBin4);
        
        servo1Label = findViewById(R.id.servo1Label);
        servo2Label = findViewById(R.id.servo2Label);
        servo3Label = findViewById(R.id.servo3Label);
        servo4Label = findViewById(R.id.servo4Label);
        servo5Label = findViewById(R.id.servo5Label);

        btnArmRotateFront = findViewById(R.id.btnArmRotateFront);
        btnArmRotateBack = findViewById(R.id.btnArmRotateBack);
        btnGripOpen = findViewById(R.id.btnGripOpen);
        btnGripClose = findViewById(R.id.btnGripClose);
        btnGripRotateMinus = findViewById(R.id.btnGripRotateMinus);
        btnGripRotatePlus = findViewById(R.id.btnGripRotatePlus);
        btnGripRotateDownDiag = findViewById(R.id.btnGripRotateDownDiag);
        btnGripRotateUpDiag = findViewById(R.id.btnGripRotateUpDiag);
        btnGripRotateVertical = findViewById(R.id.btnGripRotateVertical);
        btnGripRotateHorizontal = findViewById(R.id.btnGripRotateHorizontal);
        btnArmExtendMinus = findViewById(R.id.btnArmExtendMinus);
        btnArmExtendPlus = findViewById(R.id.btnArmExtendPlus);
        btnArmExtend180 = findViewById(R.id.btnArmExtend180);
        btnArmExtend110 = findViewById(R.id.btnArmExtend110);
        btnArmExtend90 = findViewById(R.id.btnArmExtend90);
        btnLookMinus = findViewById(R.id.btnLookMinus);
        btnLookPlus = findViewById(R.id.btnLookPlus);
        
        // Object detection buttons
        btnDetectPlasticBottle = findViewById(R.id.btnDetectPlasticBottle);
        btnDetectPlasticWrapper = findViewById(R.id.btnDetectPlasticWrapper);
        btnDetectPaper = findViewById(R.id.btnDetectPaper);
        btnDetectOther = findViewById(R.id.btnDetectOther);
        btnDetectClear = findViewById(R.id.btnDetectClear);

        // Initialize labels to match Arduino defaults immediately
        servo1Label.setText(String.format(Locale.US, "ARM-ROTATE (S1): %d°", servo1Pos));
        servo2Label.setText(String.format(Locale.US, "GRIP (S2): %d°", servo2Pos));
        servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
        servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
        servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
    }

    private void setupLifterControls() {
        btnLifterUp.setOnClickListener(v -> sendLifterCommand("UP"));
        btnLifterDown.setOnClickListener(v -> sendLifterCommand("DOWN"));
        btnLifterStop.setOnClickListener(v -> sendLifterCommand("STOP"));
    }
    
    private void setupMotorControls() {
        // Forward button - hold to move
        btnForward.setOnTouchListener((v, event) -> {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                sendMotorCommand("FORWARD", 255);
                return true;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                sendMotorCommand("STOP", 0);
                v.performClick();
                return true;
            }
            return false;
        });
        
        // Backward button - hold to move
        btnBackward.setOnTouchListener((v, event) -> {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                sendMotorCommand("BACKWARD", 255);
                return true;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                sendMotorCommand("STOP", 0);
                v.performClick();
                return true;
            }
            return false;
        });
        
        // Left button - hold to turn
        btnLeft.setOnTouchListener((v, event) -> {
            Log.d("RoboSort", "Left button touch event: " + event.getAction());
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                Log.d("RoboSort", "Left button: ACTION_DOWN - sending TURN_LEFT");
                sendMotorCommand("TURN_LEFT", 255);
                return true;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                Log.d("RoboSort", "Left button: ACTION_UP - sending STOP");
                sendMotorCommand("STOP", 0);
                v.performClick();
                return true;
            }
            return false;
        });
        
        // Right button - hold to turn
        btnRight.setOnTouchListener((v, event) -> {
            Log.d("RoboSort", "Right button touch event: " + event.getAction());
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                Log.d("RoboSort", "Right button: ACTION_DOWN - sending TURN_RIGHT");
                sendMotorCommand("TURN_RIGHT", 255);
                return true;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                Log.d("RoboSort", "Right button: ACTION_UP - sending STOP");
                sendMotorCommand("STOP", 0);
                v.performClick();
                return true;
            }
            return false;
        });
        
        // Stop button - immediate stop
        btnStop.setOnClickListener(v -> sendMotorCommand("STOP", 0));
    }
    
    private void setupBinControls() {
        btnBinHome.setOnClickListener(v -> sendBinCommand("BIN_HOME"));
        btnBin1.setOnClickListener(v -> sendBinCommand("BIN_1"));
        btnBin2.setOnClickListener(v -> sendBinCommand("BIN_2"));
        btnBin3.setOnClickListener(v -> sendBinCommand("BIN_3"));
        btnBin4.setOnClickListener(v -> sendBinCommand("BIN_4"));
    }
    
    private void setupServoControls() {
        // ARM-ROTATE: FRONT/BACK
        btnArmRotateFront.setOnClickListener(v -> {
            servo1Pos = 180;
            servo1Label.setText(String.format(Locale.US, "ARM-ROTATE (S1): %d°", servo1Pos));
            sendServoCommand(1, servo1Pos);
        });

        btnArmRotateBack.setOnClickListener(v -> {
            servo1Pos = 0;
            servo1Label.setText(String.format(Locale.US, "ARM-ROTATE (S1): %d°", servo1Pos));
            sendServoCommand(1, servo1Pos);
        });

        // GRIP: OPEN/CLOSE (fixed angles)
        btnGripOpen.setOnClickListener(v -> {
            servo2Pos = 110;
            servo2Label.setText(String.format(Locale.US, "GRIP (S2): %d°", servo2Pos));
            sendServoCommand(2, servo2Pos);
        });

        btnGripClose.setOnClickListener(v -> {
            servo2Pos = 180;
            servo2Label.setText(String.format(Locale.US, "GRIP (S2): %d°", servo2Pos));
            sendServoCommand(2, servo2Pos);
        });

        // GRIP-ROTATE: +/-
        btnGripRotateMinus.setOnClickListener(v -> {
            servo3Pos -= STEP_GRIP_ROTATE;
            servo3Pos = Math.max(0, Math.min(180, servo3Pos));
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        btnGripRotatePlus.setOnClickListener(v -> {
            servo3Pos += STEP_GRIP_ROTATE;
            servo3Pos = Math.max(0, Math.min(180, servo3Pos));
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        // GRIP-ROTATE presets: \ / | —
        btnGripRotateDownDiag.setOnClickListener(v -> {
            servo3Pos = 40;
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        btnGripRotateUpDiag.setOnClickListener(v -> {
            servo3Pos = 120;
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        btnGripRotateVertical.setOnClickListener(v -> {
            servo3Pos = 70;
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        btnGripRotateHorizontal.setOnClickListener(v -> {
            servo3Pos = 170;
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        // ARM-EXTEND: +/-
        btnArmExtendMinus.setOnClickListener(v -> {
            servo4Pos -= STEP_ARM_EXTEND;
            servo4Pos = Math.max(90, Math.min(180, servo4Pos));
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });

        btnArmExtendPlus.setOnClickListener(v -> {
            servo4Pos += STEP_ARM_EXTEND;
            servo4Pos = Math.max(90, Math.min(180, servo4Pos));
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });
        
        // ARM-EXTEND Presets: 180°, 110°, 90°
        btnArmExtend180.setOnClickListener(v -> {
            servo4Pos = 180;
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });
        
        btnArmExtend110.setOnClickListener(v -> {
            servo4Pos = 110;
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });
        
        btnArmExtend90.setOnClickListener(v -> {
            servo4Pos = 90;
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });

        // LOOK: +/-
        btnLookMinus.setOnClickListener(v -> {
            servo5Pos -= STEP_LOOK;
            servo5Pos = Math.max(0, Math.min(180, servo5Pos));
            servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
            sendServoCommand(5, servo5Pos);
        });

        btnLookPlus.setOnClickListener(v -> {
            servo5Pos += STEP_LOOK;
            servo5Pos = Math.max(0, Math.min(180, servo5Pos));
            servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
            sendServoCommand(5, servo5Pos);
        });

    }
    
    private void setupObjectDetectionControls() {
        // Object detection buttons
        btnDetectPlasticBottle.setOnClickListener(v -> sendDetectedObjectCommand("plastic_bottle"));
        btnDetectPlasticWrapper.setOnClickListener(v -> sendDetectedObjectCommand("plastic_wrapper"));
        btnDetectPaper.setOnClickListener(v -> sendDetectedObjectCommand("paper"));
        btnDetectOther.setOnClickListener(v -> sendDetectedObjectCommand("other"));
        btnDetectClear.setOnClickListener(v -> sendDetectedObjectCommand("none"));
    }
    
    private void sendDetectedObjectCommand(String objectType) {
        if (!isConnected) {
            feedbackText.setText("Cannot send DETECTED command: Not connected to Firebase");
            return;
        }

        Map<String, Object> command = new HashMap<>();
        command.put("type", "detected");
        command.put("object", objectType);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Sending DETECTED: " + objectType);

        commandsRef.child("detected").setValue(command)
            .addOnSuccessListener(aVoid -> {
                Log.d("RoboSort", "DETECTED command sent successfully: " + objectType);
                feedbackText.setText("DETECTED sent: " + objectType);
            })
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send DETECTED command: " + e.getMessage());
                feedbackText.setText("Failed to send DETECTED command: " + e.getMessage());
            });
    }
    
    private void sendBinCommand(String binCommand) {
        if (!isConnected) {
            feedbackText.setText("Cannot send BIN command: Not connected to Firebase");
            return;
        }

        Map<String, Object> command = new HashMap<>();
        command.put("type", "bin");
        command.put("command", binCommand);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Sending BIN command: " + binCommand);

        commandsRef.child("bin").setValue(command)
            .addOnSuccessListener(aVoid -> {
                Log.d("RoboSort", "BIN command sent successfully: " + binCommand);
                feedbackText.setText("BIN command sent: " + binCommand);
            })
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send BIN command: " + e.getMessage());
                feedbackText.setText("Failed to send BIN command: " + e.getMessage());
            });
    }

    private void sendLifterCommand(String action) {
        if (!isConnected) {
            feedbackText.setText("Cannot send LIFTER command: Not connected to Firebase");
            return;
        }

        String normalized = action == null ? "" : action.trim().toUpperCase(Locale.US);
        if (!normalized.equals("UP") && !normalized.equals("DOWN") && !normalized.equals("STOP")) {
            feedbackText.setText("Invalid LIFTER command: " + action);
            return;
        }

        Map<String, Object> command = new HashMap<>();
        command.put("type", "lifter");
        command.put("action", normalized);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Sending LIFTER: " + normalized);

        commandsRef.child("lifter").setValue(command)
            .addOnSuccessListener(aVoid -> Log.d("RoboSort", "LIFTER command sent: " + normalized))
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send LIFTER command: " + e.getMessage());
                feedbackText.setText("Failed to send LIFTER command: " + e.getMessage());
            });
    }

    private void sendMotorCommand(String direction, int speed) {
        if (!isConnected) {
            feedbackText.setText("Cannot send motor command: Not connected to Firebase");
            return;
        }

        String normalized = direction == null ? "STOP" : direction.trim().toUpperCase(Locale.US);
        int clampedSpeed = Math.max(0, Math.min(255, speed));

        Map<String, Object> command = new HashMap<>();
        // RPi listener accepts a motor payload with {direction, speed}
        command.put("direction", normalized);
        command.put("speed", clampedSpeed);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Motor: " + normalized + " (" + clampedSpeed + ")");

        commandsRef.child("motor").setValue(command)
            .addOnSuccessListener(aVoid -> Log.d("RoboSort", "Motor command sent: " + normalized))
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send motor command: " + e.getMessage());
                feedbackText.setText("Failed to send motor command: " + e.getMessage());
            });
    }

    private void sendServoCommand(int servoNumber, int angle) {
        if (!isConnected) {
            feedbackText.setText("Cannot send servo command: Not connected to Firebase");
            return;
        }

        int clampedAngle;
        if (servoNumber == 4) {
            // ARM-EXTEND range: 0-180 degrees
            clampedAngle = clamp(angle, 0, 180);
        } else if (servoNumber == 2) {
            // GRIP defaults in this project generally use 110-180
            clampedAngle = clamp(angle, 110, 180);
        } else {
            clampedAngle = clamp(angle, 0, 180);
        }

        Map<String, Object> command = new HashMap<>();
        // Legacy schema supported by RPi: {type:'servo', servo:n, angle:x}
        command.put("type", "servo");
        command.put("servo", servoNumber);
        command.put("angle", clampedAngle);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Servo " + servoNumber + " -> " + clampedAngle + "°");

        commandsRef.child("servo" + servoNumber).setValue(command)
            .addOnSuccessListener(aVoid -> Log.d("RoboSort", "Servo command sent: servo" + servoNumber + "=" + clampedAngle))
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send servo command: " + e.getMessage());
                feedbackText.setText("Failed to send servo command: " + e.getMessage());
            });
    }

    private void setupFeedbackListener() {
        if (feedbackRef == null) return;

        feedbackRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {
                Object raw = snapshot.getValue();
                if (!(raw instanceof Map)) {
                    runOnUiThread(() -> feedbackText.setText("No status data"));
                    return;
                }

                try {
                    //noinspection unchecked
                    Map<String, Object> feedback = (Map<String, Object>) raw;
                    displayFeedback(feedback);
                } catch (Exception e) {
                    Log.e("RoboSort", "Failed to parse feedback: " + e.getMessage());
                }
            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                Log.e("RoboSort", "Feedback listener cancelled: " + error.getMessage());
                runOnUiThread(() -> feedbackText.setText("Feedback error: " + error.getMessage()));
            }
        });
    }
    
    private void displayFeedback(Map<String, Object> feedback) {
        StringBuilder sb = new StringBuilder();
        
        // Format timestamp
        Long timestamp = (Long) feedback.get("timestamp");
        if (timestamp != null) {
            SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss", Locale.US);
            sb.append("Time: ").append(sdf.format(new Date(timestamp))).append("\n");
        }
        
        // Status
        String status = (String) feedback.get("status");
        if (status != null) {
            sb.append("Status: ").append(status).append("\n");
        }
        
        // Motor info
        String motorState = (String) feedback.get("motor_state");
        if (motorState != null) {
            sb.append("Motor: ").append(motorState).append("\n");
        }

        // Live servo positions from robosort/status (published by RPi)
        applyServoStatusIfPresent(feedback, "servo1", 1);
        applyServoStatusIfPresent(feedback, "servo2", 2);
        applyServoStatusIfPresent(feedback, "servo3", 3);
        applyServoStatusIfPresent(feedback, "servo4", 4);
        applyServoStatusIfPresent(feedback, "servo5", 5);
        
        // Servo positions
        Map<String, Object> servoPositions = (Map<String, Object>) feedback.get("servo_positions");
        if (servoPositions != null) {
            sb.append("Servos: ");
            for (Map.Entry<String, Object> entry : servoPositions.entrySet()) {
                sb.append(entry.getKey()).append("=").append(entry.getValue()).append("° ");
            }
            sb.append("\n");
        }
        
        // Sensors
        Map<String, Object> sensors = (Map<String, Object>) feedback.get("sensors");
        if (sensors != null) {
            Object distance = sensors.get("ultrasonic_distance");
            if (distance != null) {
                sb.append("Distance: ").append(distance).append(" cm\n");
            }
        }
        
        // Error
        String error = (String) feedback.get("error");
        if (error != null && !error.isEmpty()) {
            sb.append("⚠ Error: ").append(error).append("\n");
        }
        
        final String feedbackStr = sb.toString();
        runOnUiThread(() -> feedbackText.setText(feedbackStr));
    }

    private void applyServoStatusIfPresent(Map<String, Object> status, String key, int servoNum) {
        Object raw = status.get(key);
        if (raw == null) return;

        int angle;
        if (raw instanceof Long) {
            angle = ((Long) raw).intValue();
        } else if (raw instanceof Integer) {
            angle = (Integer) raw;
        } else if (raw instanceof Double) {
            angle = ((Double) raw).intValue();
        } else {
            return;
        }

        angle = clamp(angle, 0, 180);

        switch (servoNum) {
            case 1:
                servo1Pos = angle;
                servo1Label.setText(String.format(Locale.US, "ARM-ROTATE (S1): %d°", angle));
                break;
            case 2:
                servo2Pos = clamp(angle, 110, 180);
                servo2Label.setText(String.format(Locale.US, "GRIP (S2): %d°", servo2Pos));
                break;
            case 3:
                servo3Pos = angle;
                servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", angle));
                break;
            case 4:
                servo4Pos = clamp(angle, 0, 180);
                servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
                break;
            case 5:
                servo5Pos = angle;
                servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", angle));
                break;
        }
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private void updateConnectionStatus(boolean connected) {
        isConnected = connected;
        runOnUiThread(() -> {
            if (connected) {
                statusText.setText("● Connected");
                statusText.setTextColor(getResources().getColor(android.R.color.holo_green_dark));
            } else {
                statusText.setText("● Disconnected");
                statusText.setTextColor(getResources().getColor(android.R.color.holo_red_dark));
            }
        });
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Send stop command before closing
        if (isConnected) {
            sendMotorCommand("STOP", 0);
        }

    }
}
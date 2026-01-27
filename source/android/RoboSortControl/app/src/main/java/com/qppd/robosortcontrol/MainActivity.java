package com.qppd.robosortcontrol;

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
import com.google.android.material.slider.Slider;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

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
    private TextView servo1Label, servo2Label, servo3Label, servo4Label;
    
    private MaterialButton btnForward, btnBackward, btnLeft, btnRight, btnStop;
    private Slider servo1Slider, servo2Slider, servo3Slider, servo4Slider;
    
    // State
    private boolean isConnected = false;
    private Handler handler;
    
    // Servo positions
    // Match Arduino SERVO_CONFIG defaults (channels 1-4)
    private int servo1Pos = 180;
    private int servo2Pos = 105;
    private int servo3Pos = 90;
    private int servo4Pos = 90;

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
        setupServoControls();
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
        
        servo1Label = findViewById(R.id.servo1Label);
        servo2Label = findViewById(R.id.servo2Label);
        servo3Label = findViewById(R.id.servo3Label);
        servo4Label = findViewById(R.id.servo4Label);
        
        servo1Slider = findViewById(R.id.servo1Slider);
        servo2Slider = findViewById(R.id.servo2Slider);
        servo3Slider = findViewById(R.id.servo3Slider);
        servo4Slider = findViewById(R.id.servo4Slider);

        // Initialize sliders/labels to match Arduino defaults immediately
        servo1Slider.setValue(servo1Pos);
        servo2Slider.setValue(servo2Pos);
        servo3Slider.setValue(servo3Pos);
        servo4Slider.setValue(servo4Pos);

        servo1Label.setText(String.format(Locale.US, "Arm Base (S1): %d°", servo1Pos));
        servo2Label.setText(String.format(Locale.US, "Arm Shoulder (S2): %d°", servo2Pos));
        servo3Label.setText(String.format(Locale.US, "Arm Elbow (S3): %d°", servo3Pos));
        servo4Label.setText(String.format(Locale.US, "Gripper (S4): %d°", servo4Pos));
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
    
    private void setupServoControls() {
        // Servo 1 - Arm Base
        servo1Slider.addOnChangeListener((slider, value, fromUser) -> {
            if (fromUser) {
                servo1Pos = (int) value;
                servo1Label.setText(String.format(Locale.US, "Arm Base (S1): %d°", servo1Pos));
                sendServoCommand(1, servo1Pos);
            }
        });
        
        // Servo 2 - Arm Shoulder
        servo2Slider.addOnChangeListener((slider, value, fromUser) -> {
            if (fromUser) {
                servo2Pos = (int) value;
                servo2Label.setText(String.format(Locale.US, "Arm Shoulder (S2): %d°", servo2Pos));
                sendServoCommand(2, servo2Pos);
            }
        });
        
        // Servo 3 - Arm Elbow
        servo3Slider.addOnChangeListener((slider, value, fromUser) -> {
            if (fromUser) {
                servo3Pos = (int) value;
                servo3Label.setText(String.format(Locale.US, "Arm Elbow (S3): %d°", servo3Pos));
                sendServoCommand(3, servo3Pos);
            }
        });
        
        // Servo 4 - Gripper
        servo4Slider.addOnChangeListener((slider, value, fromUser) -> {
            if (fromUser) {
                servo4Pos = (int) value;
                servo4Label.setText(String.format(Locale.US, "Gripper (S4): %d°", servo4Pos));
                sendServoCommand(4, servo4Pos);
            }
        });
    }
    
    private void setupFeedbackListener() {
        feedbackRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {
                if (snapshot.exists()) {
                    Map<String, Object> feedback = (Map<String, Object>) snapshot.getValue();
                    if (feedback != null) {
                        displayFeedback(feedback);
                    }
                }
            }
            
            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                feedbackText.setText("Error reading feedback: " + error.getMessage());
            }
        });
    }
    
    private void sendMotorCommand(String direction, int speed) {
        Log.d("RoboSort", "sendMotorCommand called: direction=" + direction + ", speed=" + speed);
        if (!isConnected) {
            Log.w("RoboSort", "Cannot send command: Not connected to Firebase");
            feedbackText.setText("Cannot send command: Not connected to Firebase");
            return;
        }
        
        Map<String, Object> command = new HashMap<>();
        command.put("type", "motor");
        command.put("direction", direction);
        command.put("speed", speed);
        command.put("timestamp", System.currentTimeMillis());
        
        Log.d("RoboSort", "Sending command to Firebase: " + command);
        feedbackText.setText("Sending: " + direction + " at speed " + speed);
        
        commandsRef.child("motor").setValue(command)
            .addOnSuccessListener(aVoid -> {
                Log.d("RoboSort", "Command sent successfully: " + direction);
                feedbackText.setText("Command sent successfully: " + direction);
            })
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send command: " + e.getMessage());
                feedbackText.setText("Failed to send command: " + e.getMessage());
            });
    }
    
    private void sendServoCommand(int servoNum, int angle) {
        if (!isConnected) return;
        
        Map<String, Object> command = new HashMap<>();
        command.put("type", "servo");
        command.put("servo", servoNum);
        command.put("angle", angle);
        command.put("timestamp", System.currentTimeMillis());
        
        commandsRef.child("servo" + servoNum).setValue(command)
            .addOnFailureListener(e -> {
                runOnUiThread(() -> feedbackText.setText("Failed to send servo command: " + e.getMessage()));
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
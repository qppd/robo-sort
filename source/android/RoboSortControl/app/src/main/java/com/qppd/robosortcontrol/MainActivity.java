package com.qppd.robosortcontrol;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.widget.ImageView;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.EditText;
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
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends AppCompatActivity {

    private static final String PREFS_NAME = "robosort_prefs";
    private static final String PREF_STREAM_URL = "camera_stream_url";

    // Firebase
    private FirebaseDatabase database;
    private DatabaseReference commandsRef;
    private DatabaseReference feedbackRef;
    
    // UI Elements
    private TextView statusText;
    private TextView feedbackText;
    private TextView servo1Label, servo2Label, servo3Label, servo4Label, servo5Label;

    private EditText streamUrlInput;
    private MaterialButton btnLoadStream;
    private WebView cameraWebView;
    private ImageView cameraImageView;
    
    private MaterialButton btnForward, btnBackward, btnLeft, btnRight, btnStop;
    private MaterialButton btnLifterUp, btnLifterDown, btnLifterStop;
    private MaterialButton btnBinHome, btnBin1, btnBin2, btnBin3, btnBin4;

    // Servo buttons (replace sliders)
    private MaterialButton btnArmRotateFront, btnArmRotateBack;
    private MaterialButton btnGripOpen, btnGripClose;
    private MaterialButton btnGripRotateMinus, btnGripRotatePlus;
    private MaterialButton btnArmExtendMinus, btnArmExtendPlus;
    private MaterialButton btnLookMinus, btnLookPlus;
    
    // State
    private boolean isConnected = false;
    private Handler handler;
    private ExecutorService streamExecutor;
    private volatile boolean isStreaming = false;
    
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
        streamExecutor = Executors.newSingleThreadExecutor();
        
        // Initialize Firebase
        initializeFirebase();
        
        // Initialize UI
        initializeUI();
        
        // Setup listeners
        setupMotorControls();
        setupLifterControls();
        setupBinControls();
        setupServoControls();
        setupCameraStreamControls();
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
        btnArmExtendMinus = findViewById(R.id.btnArmExtendMinus);
        btnArmExtendPlus = findViewById(R.id.btnArmExtendPlus);
        btnLookMinus = findViewById(R.id.btnLookMinus);
        btnLookPlus = findViewById(R.id.btnLookPlus);

        streamUrlInput = findViewById(R.id.streamUrlInput);
        btnLoadStream = findViewById(R.id.btnLoadStream);
        cameraWebView = findViewById(R.id.cameraWebView);
        cameraImageView = findViewById(R.id.cameraImageView);

        // Initialize labels to match Arduino defaults immediately
        servo1Label.setText(String.format(Locale.US, "ARM-ROTATE (S1): %d°", servo1Pos));
        servo2Label.setText(String.format(Locale.US, "GRIP (S2): %d°", servo2Pos));
        servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
        servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
        servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
    }

    private void setupCameraStreamControls() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String lastUrl = prefs.getString(PREF_STREAM_URL, "");
        if (lastUrl != null && !lastUrl.trim().isEmpty()) {
            streamUrlInput.setText(lastUrl);
        }

        btnLoadStream.setOnClickListener(v -> {
            String url = streamUrlInput.getText() != null ? streamUrlInput.getText().toString().trim() : "";
            if (url.isEmpty()) {
                feedbackText.setText("Enter stream URL (example: https://xxxx.ngrok-free.app/video)");
                return;
            }

            // Stop any existing stream
            isStreaming = false;

            prefs.edit().putString(PREF_STREAM_URL, url).apply();

            // Hide WebView, show ImageView
            cameraWebView.setVisibility(View.GONE);
            cameraImageView.setVisibility(View.VISIBLE);

            // Start MJPEG streaming
            startMJPEGStream(url);
        });
    }

    private void startMJPEGStream(String url) {
        isStreaming = true;

        streamExecutor.execute(() -> {
            try {
                URL streamUrl = new URL(url);
                HttpURLConnection connection = (HttpURLConnection) streamUrl.openConnection();

                // Add ngrok bypass header if it's an ngrok URL
                if (url.contains("ngrok")) {
                    connection.setRequestProperty("ngrok-skip-browser-warning", "true");
                }

                connection.setRequestMethod("GET");
                connection.setConnectTimeout(5000);
                connection.setReadTimeout(10000);
                connection.connect();

                if (connection.getResponseCode() == HttpURLConnection.HTTP_OK) {
                    InputStream inputStream = new BufferedInputStream(connection.getInputStream());
                    byte[] buffer = new byte[4096];
                    int bytesRead;
                    StringBuilder headerBuilder = new StringBuilder();

                    while (isStreaming) {
                        // Read until we find the Content-Length header
                        headerBuilder.setLength(0);
                        boolean inHeaders = true;

                        while (inHeaders && isStreaming) {
                            bytesRead = inputStream.read();
                            if (bytesRead == -1) break;

                            char c = (char) bytesRead;
                            headerBuilder.append(c);

                            if (headerBuilder.toString().endsWith("\r\n\r\n")) {
                                inHeaders = false;
                            }
                        }

                        if (!isStreaming) break;

                        // Parse Content-Length
                        String headers = headerBuilder.toString();
                        int contentLength = -1;
                        String[] headerLines = headers.split("\r\n");
                        for (String line : headerLines) {
                            if (line.toLowerCase().startsWith("content-length:")) {
                                try {
                                    contentLength = Integer.parseInt(line.substring(15).trim());
                                } catch (NumberFormatException e) {
                                    Log.e("RoboSort", "Invalid Content-Length", e);
                                }
                                break;
                            }
                        }

                        if (contentLength > 0) {
                            // Read the JPEG data
                            byte[] jpegData = new byte[contentLength];
                            int totalRead = 0;

                            while (totalRead < contentLength && isStreaming) {
                                bytesRead = inputStream.read(jpegData, totalRead, contentLength - totalRead);
                                if (bytesRead == -1) break;
                                totalRead += bytesRead;
                            }

                            if (totalRead == contentLength && isStreaming) {
                                // Decode and display the frame
                                Bitmap bitmap = BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length);
                                if (bitmap != null) {
                                    runOnUiThread(() -> cameraImageView.setImageBitmap(bitmap));
                                }
                            }
                        }

                        // Skip to next frame boundary
                        while (isStreaming) {
                            bytesRead = inputStream.read();
                            if (bytesRead == -1) break;

                            if ((char) bytesRead == '\n') {
                                // Check for boundary
                                byte[] boundaryCheck = new byte[2];
                                int checkRead = inputStream.read(boundaryCheck);
                                if (checkRead == 2 && boundaryCheck[0] == '-' && boundaryCheck[1] == '-') {
                                    break;
                                }
                            }
                        }
                    }

                    inputStream.close();
                } else {
                    runOnUiThread(() -> {
                        try {
                            feedbackText.setText("Stream connection failed: " + connection.getResponseCode());
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    });
                }

                connection.disconnect();

            } catch (IOException e) {
                if (isStreaming) {
                    Log.e("RoboSort", "MJPEG stream error", e);
                    runOnUiThread(() -> feedbackText.setText("Stream error: " + e.getMessage()));
                }
            }
        });
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
            servo3Pos = clamp(servo3Pos - STEP_GRIP_ROTATE, 0, 180);
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        btnGripRotatePlus.setOnClickListener(v -> {
            servo3Pos = clamp(servo3Pos + STEP_GRIP_ROTATE, 0, 180);
            servo3Label.setText(String.format(Locale.US, "GRIP-ROTATE (S3): %d°", servo3Pos));
            sendServoCommand(3, servo3Pos);
        });

        // ARM-EXTEND: +/- (110..180)
        btnArmExtendMinus.setOnClickListener(v -> {
            servo4Pos = clamp(servo4Pos - STEP_ARM_EXTEND, 110, 180);
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });

        btnArmExtendPlus.setOnClickListener(v -> {
            servo4Pos = clamp(servo4Pos + STEP_ARM_EXTEND, 110, 180);
            servo4Label.setText(String.format(Locale.US, "ARM-EXTEND (S4): %d°", servo4Pos));
            sendServoCommand(4, servo4Pos);
        });

        // LOOK: +/-
        btnLookMinus.setOnClickListener(v -> {
            servo5Pos = clamp(servo5Pos - STEP_LOOK, 0, 180);
            servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
            sendServoCommand(5, servo5Pos);
        });

        btnLookPlus.setOnClickListener(v -> {
            servo5Pos = clamp(servo5Pos + STEP_LOOK, 0, 180);
            servo5Label.setText(String.format(Locale.US, "LOOK (S5): %d°", servo5Pos));
            sendServoCommand(5, servo5Pos);
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

    private void sendLifterCommand(String action) {
        if (!isConnected) {
            feedbackText.setText("Cannot send lifter command: Not connected to Firebase");
            return;
        }

        // Safety check: Prevent LIFTER DOWN if ARM-EXTEND is below 110 degrees
        if ("DOWN".equals(action)) {
            int armExtendAngle = servo4Pos;  // servo4 = ARM-EXTEND
            if (armExtendAngle < 110) {
                feedbackText.setText("⚠ SAFETY BLOCK: Cannot lower lifter - ARM-EXTEND is at " + armExtendAngle + "° (must be ≥110°). Extend arm first!");
                return;  // Don't send the command
            }
        }

        Map<String, Object> command = new HashMap<>();
        command.put("type", "lifter");
        command.put("action", action);
        command.put("timestamp", System.currentTimeMillis());

        feedbackText.setText("Sending LIFTER " + action + "...");

        commandsRef.child("lifter").setValue(command)
            .addOnSuccessListener(aVoid -> {
                Log.d("RoboSort", "Lifter command sent successfully: " + action);
                feedbackText.setText("Lifter command sent: " + action);
            })
            .addOnFailureListener(e -> {
                Log.e("RoboSort", "Failed to send lifter command: " + e.getMessage());
                feedbackText.setText("Failed to send lifter command: " + e.getMessage());
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
                servo4Pos = clamp(angle, 110, 180);
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
        // Stop streaming and shutdown executor
        isStreaming = false;
        if (streamExecutor != null && !streamExecutor.isShutdown()) {
            streamExecutor.shutdown();
        }
    }
}
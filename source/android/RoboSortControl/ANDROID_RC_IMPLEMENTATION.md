# Android Real-Time RC Control Implementation

## MainActivity.java - Complete Implementation

```java
package com.qppd.robosortcontrol;

import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ServerValue;
import com.google.firebase.database.ValueEventListener;

public class MainActivity extends AppCompatActivity {

    // Firebase
    private FirebaseDatabase database;
    private DatabaseReference commandsRef;
    private DatabaseReference statusRef;
    
    // UI Elements
    private Button btnForward, btnBackward, btnLeft, btnRight;
    private SeekBar seekServo1, seekServo2, seekServo3, seekServo4;
    private TextView tvMotorStatus, tvServo1, tvServo2, tvServo3, tvServo4, tvConnectionStatus;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        // Initialize Firebase
        database = FirebaseDatabase.getInstance();
        commandsRef = database.getReference("robosort/commands");
        statusRef = database.getReference("robosort/status");
        
        // Initialize UI
        initializeViews();
        setupMotorControls();
        setupServoControls();
        listenToStatus();
    }
    
    private void initializeViews() {
        // Motor control buttons
        btnForward = findViewById(R.id.btnForward);
        btnBackward = findViewById(R.id.btnBackward);
        btnLeft = findViewById(R.id.btnLeft);
        btnRight = findViewById(R.id.btnRight);
        
        // Servo seekbars
        seekServo1 = findViewById(R.id.seekServo1);
        seekServo2 = findViewById(R.id.seekServo2);
        seekServo3 = findViewById(R.id.seekServo3);
        seekServo4 = findViewById(R.id.seekServo4);
        
        // Status TextViews
        tvMotorStatus = findViewById(R.id.tvMotorStatus);
        tvServo1 = findViewById(R.id.tvServo1);
        tvServo2 = findViewById(R.id.tvServo2);
        tvServo3 = findViewById(R.id.tvServo3);
        tvServo4 = findViewById(R.id.tvServo4);
        tvConnectionStatus = findViewById(R.id.tvConnectionStatus);
        
        // Set seekbar ranges (0-180)
        seekServo1.setMax(180);
        seekServo2.setMax(180);
        seekServo3.setMax(180);
        seekServo4.setMax(180);
        
        // Set initial positions
        seekServo1.setProgress(90);
        seekServo2.setProgress(90);
        seekServo3.setProgress(90);
        seekServo4.setProgress(90);
    }
    
    private void setupMotorControls() {
        // FORWARD button - RC style (press = move, release = stop)
        btnForward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        sendMotorCommand("FORWARD");
                        v.setPressed(true);
                        return true;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        sendMotorCommand("STOP");
                        v.setPressed(false);
                        v.performClick();
                        return true;
                }
                return false;
            }
        });
        
        // BACKWARD button
        btnBackward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        sendMotorCommand("BACKWARD");
                        v.setPressed(true);
                        return true;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        sendMotorCommand("STOP");
                        v.setPressed(false);
                        v.performClick();
                        return true;
                }
                return false;
            }
        });
        
        // LEFT button
        btnLeft.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        sendMotorCommand("TURN_LEFT");
                        v.setPressed(true);
                        return true;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        sendMotorCommand("STOP");
                        v.setPressed(false);
                        v.performClick();
                        return true;
                }
                return false;
            }
        });
        
        // RIGHT button
        btnRight.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        sendMotorCommand("TURN_RIGHT");
                        v.setPressed(true);
                        return true;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        sendMotorCommand("STOP");
                        v.setPressed(false);
                        v.performClick();
                        return true;
                }
                return false;
            }
        });
    }
    
    private void setupServoControls() {
        // Servo 1
        seekServo1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    sendServoCommand("servo1", progress);
                    tvServo1.setText("Servo 1: " + progress + "°");
                }
            }
            
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        
        // Servo 2
        seekServo2.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    sendServoCommand("servo2", progress);
                    tvServo2.setText("Servo 2: " + progress + "°");
                }
            }
            
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        
        // Servo 3
        seekServo3.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    sendServoCommand("servo3", progress);
                    tvServo3.setText("Servo 3: " + progress + "°");
                }
            }
            
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        
        // Servo 4
        seekServo4.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    sendServoCommand("servo4", progress);
                    tvServo4.setText("Servo 4: " + progress + "°");
                }
            }
            
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
    }
    
    private void sendMotorCommand(String command) {
        commandsRef.child("motor").setValue(command);
        commandsRef.child("timestamp").setValue(ServerValue.TIMESTAMP);
    }
    
    private void sendServoCommand(String servoName, int angle) {
        commandsRef.child(servoName).setValue(angle);
        commandsRef.child("timestamp").setValue(ServerValue.TIMESTAMP);
    }
    
    private void listenToStatus() {
        statusRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(DataSnapshot dataSnapshot) {
                // Motor state
                String motorState = dataSnapshot.child("motor_state").getValue(String.class);
                if (motorState != null) {
                    tvMotorStatus.setText("Motor: " + motorState);
                }
                
                // Connection status
                Boolean connected = dataSnapshot.child("connected").getValue(Boolean.class);
                if (connected != null && connected) {
                    tvConnectionStatus.setText("Connected");
                    tvConnectionStatus.setTextColor(getResources().getColor(android.R.color.holo_green_dark));
                } else {
                    tvConnectionStatus.setText("Disconnected");
                    tvConnectionStatus.setTextColor(getResources().getColor(android.R.color.holo_red_dark));
                }
                
                // Servo positions (update from feedback)
                Integer servo1 = dataSnapshot.child("servo1").getValue(Integer.class);
                if (servo1 != null) {
                    tvServo1.setText("Servo 1: " + servo1 + "°");
                }
                
                Integer servo2 = dataSnapshot.child("servo2").getValue(Integer.class);
                if (servo2 != null) {
                    tvServo2.setText("Servo 2: " + servo2 + "°");
                }
                
                Integer servo3 = dataSnapshot.child("servo3").getValue(Integer.class);
                if (servo3 != null) {
                    tvServo3.setText("Servo 3: " + servo3 + "°");
                }
                
                Integer servo4 = dataSnapshot.child("servo4").getValue(Integer.class);
                if (servo4 != null) {
                    tvServo4.setText("Servo 4: " + servo4 + "°");
                }
            }
            
            @Override
            public void onCancelled(DatabaseError databaseError) {
                tvConnectionStatus.setText("Error: " + databaseError.getMessage());
                tvConnectionStatus.setTextColor(getResources().getColor(android.R.color.holo_red_dark));
            }
        });
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Send stop command on exit
        sendMotorCommand("STOP");
    }
}
```

## activity_main.xml Layout

```xml
<?xml version="1.0" encoding="utf-8"?>
<LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:orientation="vertical"
    android:padding="16dp">

    <!-- Connection Status -->
    <TextView
        android:id="@+id/tvConnectionStatus"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Connecting..."
        android:textSize="18sp"
        android:textStyle="bold"
        android:gravity="center"
        android:padding="8dp"/>

    <!-- Motor Status -->
    <TextView
        android:id="@+id/tvMotorStatus"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Motor: STOP"
        android:textSize="16sp"
        android:gravity="center"
        android:padding="8dp"/>

    <!-- Motor Controls -->
    <TextView
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Motor Controls"
        android:textSize="18sp"
        android:textStyle="bold"
        android:paddingTop="16dp"
        android:paddingBottom="8dp"/>

    <LinearLayout
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:orientation="vertical"
        android:gravity="center">

        <!-- Forward -->
        <Button
            android:id="@+id/btnForward"
            android:layout_width="120dp"
            android:layout_height="60dp"
            android:text="FORWARD"
            android:textSize="12sp"/>

        <!-- Left and Right -->
        <LinearLayout
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:orientation="horizontal"
            android:layout_marginTop="8dp">

            <Button
                android:id="@+id/btnLeft"
                android:layout_width="120dp"
                android:layout_height="60dp"
                android:text="LEFT"
                android:textSize="12sp"
                android:layout_marginEnd="16dp"/>

            <Button
                android:id="@+id/btnRight"
                android:layout_width="120dp"
                android:layout_height="60dp"
                android:text="RIGHT"
                android:textSize="12sp"/>
        </LinearLayout>

        <!-- Backward -->
        <Button
            android:id="@+id/btnBackward"
            android:layout_width="120dp"
            android:layout_height="60dp"
            android:text="BACKWARD"
            android:textSize="12sp"
            android:layout_marginTop="8dp"/>
    </LinearLayout>

    <!-- Servo Controls -->
    <TextView
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Servo Controls"
        android:textSize="18sp"
        android:textStyle="bold"
        android:paddingTop="24dp"
        android:paddingBottom="8dp"/>

    <!-- Servo 1 -->
    <TextView
        android:id="@+id/tvServo1"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Servo 1: 90°"
        android:textSize="14sp"/>
    <SeekBar
        android:id="@+id/seekServo1"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:layout_marginBottom="8dp"/>

    <!-- Servo 2 -->
    <TextView
        android:id="@+id/tvServo2"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Servo 2: 90°"
        android:textSize="14sp"/>
    <SeekBar
        android:id="@+id/seekServo2"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:layout_marginBottom="8dp"/>

    <!-- Servo 3 -->
    <TextView
        android:id="@+id/tvServo3"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Servo 3: 90°"
        android:textSize="14sp"/>
    <SeekBar
        android:id="@+id/seekServo3"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:layout_marginBottom="8dp"/>

    <!-- Servo 4 -->
    <TextView
        android:id="@+id/tvServo4"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Servo 4: 90°"
        android:textSize="14sp"/>
    <SeekBar
        android:id="@+id/seekServo4"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"/>

</LinearLayout>
```

## Key Implementation Details

### RC-Style Button Behavior
- Uses `OnTouchListener` instead of `OnClickListener`
- `ACTION_DOWN`: Send movement command
- `ACTION_UP` / `ACTION_CANCEL`: Send STOP command
- This mimics real RC controller behavior

### Real-Time Updates
- `ValueEventListener` on status node provides instant feedback
- No polling, pure push-based updates from Firebase
- UI updates automatically when robot state changes

### No Delays
- All Firebase writes are asynchronous (non-blocking)
- No sleep() or Thread.sleep() anywhere
- Instant command transmission

### Safety Feature
- `onDestroy()` sends STOP command when app closes
- Prevents runaway robot if app crashes or closes

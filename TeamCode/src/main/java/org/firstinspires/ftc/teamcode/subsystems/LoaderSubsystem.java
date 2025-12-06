package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoaderSubsystem {
    // hardware
    private final Servo servoOne;
    private final ElapsedTime cycleTimer = new ElapsedTime();

    // crop range on the physical servo
    private static final double SERVO_MIN = 0.32;
    private static final double SERVO_MAX = 0.55;

    // logical positions (0–1 BEFORE scaling)
    private static final double DEFAULT_POS = 0.0; // -> 0.3 physical
    private static final double UP_POS      = 1.0; // -> 0.7 physical

    // time to stay "up"
    private static final double UP_TIME_SEC = 0.10;

    private boolean isCycling = false;

    public LoaderSubsystem(HardwareMap hardwareMap) {
        servoOne = hardwareMap.get(Servo.class, "servo_one");
        servoOne.scaleRange(SERVO_MIN, SERVO_MAX);
        servoOne.setPosition(DEFAULT_POS);
    }

    public void setDefault() {
        servoOne.setPosition(DEFAULT_POS);
    }

    public void setUp() {
        servoOne.setPosition(UP_POS);
    }

    public void toggle() {
        if (servoOne.getPosition() < 0.5) {
            servoOne.setPosition(UP_POS);
        } else {
            servoOne.setPosition(DEFAULT_POS);
        }
    }

    // Start one up-then-down cycle
    public void startCycle() {
        if (!isCycling) {
            isCycling = true;
            servoOne.setPosition(UP_POS);   // go up first
            cycleTimer.reset();             // start timing
        }
    }

    // Call this every loop
    public void updateLoader() {
        if (isCycling) {
            if (cycleTimer.seconds() >= UP_TIME_SEC) {
                servoOne.setPosition(DEFAULT_POS);  // go back down
                isCycling = false;                  // cycle done
            }
        }
    }

    public boolean isCycling() {
        return isCycling;
    }
}

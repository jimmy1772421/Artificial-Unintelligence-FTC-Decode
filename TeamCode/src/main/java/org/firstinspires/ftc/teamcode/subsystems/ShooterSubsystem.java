package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    // goBILDA 6000 RPM motor
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    // Internal helpers
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private final DcMotorEx motor;
    private boolean isOn = false;

    // Target speed controlled ONLY by buttons
    private double targetRpm = 0.0; // start at 0; bump with buttons

    // Edge detection
    private boolean lastToggleBtn = false;
    private boolean lastIncBtn = false;
    private boolean lastDecBtn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Call this once per loop with the three buttons you map:
     * @param togglePressed      e.g., gamepad1.a (rising edge toggles on/off)
     * @param increasePressed    e.g., gamepad1.dpad_up (rising edge +250 RPM)
     * @param decreasePressed    e.g., gamepad1.dpad_down (rising edge -250 RPM)
     */
    public void update(boolean togglePressed, boolean increasePressed, boolean decreasePressed) {
        // Toggle on rising edge
        if (togglePressed && !lastToggleBtn) {
            isOn = !isOn;
        }

        // Adjust target RPM on rising edges
        if (increasePressed && !lastIncBtn) {
            targetRpm = clampRpm(targetRpm + RPM_STEP);
        }
        if (decreasePressed && !lastDecBtn) {
            targetRpm = clampRpm(targetRpm - RPM_STEP);
        }

        // Apply output
        if (isOn) {
            motor.setVelocity(rpmToTicksPerSec(targetRpm));
        } else {
            motor.setVelocity(0.0);
        }

        // Latch buttons
        lastToggleBtn = togglePressed;
        lastIncBtn = increasePressed;
        lastDecBtn = decreasePressed;
    }

    private double clampRpm(double rpm) {
        if (rpm < 0) return 0;
        if (rpm > PHYSICAL_MAX_RPM) return PHYSICAL_MAX_RPM;
        return rpm;
    }

    // Telemetry helpers
    public boolean isOn() { return isOn; }
    public double getTargetRpm() { return targetRpm; }
    public double getCurrentRpmEstimate() { return ticksPerSecToRpm(motor.getVelocity()); }

    public void stop() {
        isOn = false;
        targetRpm = 0.0;
        motor.setVelocity(0.0);
    }
}

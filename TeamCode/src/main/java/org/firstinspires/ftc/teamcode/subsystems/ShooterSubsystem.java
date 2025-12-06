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

    // Target speed controlled by dpad
    private double targetRpm = 3800; // near-field default

    // Edge detection for RPM buttons
    private boolean lastIncBtn = false;
    private boolean lastDecBtn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Call this once per loop.
     *
     * @param enabled          true = shooter should be running, false = off
     * @param increasePressed  e.g. gamepad1.dpad_up  (rising edge +250 RPM)
     * @param decreasePressed  e.g. gamepad1.dpad_down (rising edge -250 RPM)
     * @param fieldPosition    0 = near, 1 = far, anything else = safety off
     */
    public void update(boolean enabled,
                       boolean increasePressed,
                       boolean decreasePressed,
                       int fieldPosition) {

        // Directly set on/off from caller (spindexer / driver logic)
        isOn = enabled;

        // Adjust target RPM on rising edges of dpad up/down
        if (increasePressed && !lastIncBtn) {
            targetRpm = clampRpm(targetRpm + RPM_STEP);
        }
        if (decreasePressed && !lastDecBtn) {
            targetRpm = clampRpm(targetRpm - RPM_STEP);
        }

        // Apply output
        if (isOn) {
            double rpmCommand;

            if (fieldPosition == 0) {
                // near field
                rpmCommand = targetRpm;
            } else if (fieldPosition == 1) {
                // far field: your original offset (4750 - 3800)
                rpmCommand = targetRpm + (4750.0 - 3800.0);
            } else {
                // unknown field position, be safe
                rpmCommand = 0.0;
            }

            motor.setVelocity(rpmToTicksPerSec(rpmCommand));
        } else {
            motor.setVelocity(0.0);
        }

        // Latch buttons for edge detection
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
        motor.setVelocity(0.0);
    }
}

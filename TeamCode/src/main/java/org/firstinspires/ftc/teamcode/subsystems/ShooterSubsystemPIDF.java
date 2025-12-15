package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystemPIDF {

    // ===== MOTOR CONSTANTS =====
    // goBILDA 6000 RPM motor
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;
    private static final double MAX_RPM = PHYSICAL_MAX_RPM;
    private static final double MAX_TICKS_PER_SEC = PHYSICAL_MAX_RPM * TICKS_PER_REV / 60.0;

    // ===== HOOD SERVO CONSTANTS =====
    // Adjust in practice:
    //  0.85 -> ~35 degrees (near)
    //  0.92 -> ~60 degrees (far-ish)
    private static final double HOOD_NEAR_POS = 0.85;
    private static final double HOOD_FAR_POS  = 0.92;

    // ===== HELPERS =====
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    // ===== HARDWARE =====
    private final DcMotorEx motor;
    private final Servo hoodServo;
    private final LightSubsystem light;

    // ===== STATE =====
    private boolean isOn = false;   // shooter commanded on/off
    private int fieldPos = 0;       // 0 = nearfield, 1 = far

    private double nearRpm = 2700.0;   // tune these on field
    private double farRpm  = 3300.0;

    // edge-detection for RPM buttons
    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== PIDF GAINS (for built-in motor controller) =====
    // These are your logical gains; we push them into the motor via setPIDFCoefficients
    private double kP = 0.0008;
    private double kI = 0.0004;
    private double kD = 0.0002;
    // Feedforward: power ≈ kF * targetTicksPerSec, so kF ≈ 1 / maxTicksPerSec
    private double kF = 1.0 / MAX_TICKS_PER_SEC;

    public ShooterSubsystemPIDF(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        light = new LightSubsystem(hardwareMap);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        applyPidfToMotor(); // push initial gains into the motor

        // Start NEAR field by default
        fieldPos = 0;
        hoodServo.setPosition(HOOD_NEAR_POS);
    }

    /** Push current kP/kI/kD/kF to the motor's internal PIDF controller. */
    private void applyPidfToMotor() {
        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    /**
     * Call this once per loop from TeleOp.
     *
     * @param shooterOnCommand  True = run shooter, False = stop
     * @param rpmUpButton       D-pad up (rising edge bumps RPM for current fieldPos)
     * @param rpmDownButton     D-pad down (rising edge lowers RPM for current fieldPos)
     * @param fieldPosInput     0 = near, 1 = far
     */
    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        // === FIELD POSITION / HOOD CONTROL ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;

        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;
            if (fieldPos == 1) {
                hoodServo.setPosition(HOOD_FAR_POS);
            } else {
                hoodServo.setPosition(HOOD_NEAR_POS);
            }
        }

        // === RPM ADJUSTMENT (per mode) ===
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) {
                farRpm += RPM_STEP;
                if (farRpm > PHYSICAL_MAX_RPM) farRpm = PHYSICAL_MAX_RPM;
            } else {
                nearRpm += RPM_STEP;
                if (nearRpm > PHYSICAL_MAX_RPM) nearRpm = PHYSICAL_MAX_RPM;
            }
        }

        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) {
                farRpm -= RPM_STEP;
                if (farRpm < 0) farRpm = 0;
            } else {
                nearRpm -= RPM_STEP;
                if (nearRpm < 0) nearRpm = 0;
            }
        }

        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY SHOOTER COMMAND ===
        isOn = shooterOnCommand;

        double targetRpm = getTargetRpm();

        if (isOn && targetRpm > 0) {
            // Use built-in PIDF velocity control in ticks/second
            double targetTps = rpmToTicksPerSec(targetRpm);
            motor.setVelocity(targetTps);

            // Compute error just for lights / telemetry
            double currentRpm = getCurrentRpmEstimate();
            double error = targetRpm - currentRpm;

            if (Math.abs(error) < 200) {
                light.setColor(3);   // ready
            } else {
                light.setColor(2);   // spinning but not yet at speed
            }
        } else {
            stop();
            light.setColor(1);       // off
        }
    }

    // ===== GETTERS =====

    public boolean isOn() {
        return isOn;
    }

    public int getFieldPos() {
        return fieldPos;
    }

    public double getNearRpm() {
        return nearRpm;
    }

    public double getFarRpm() {
        return farRpm;
    }

    public double getTargetRpm() {
        return (fieldPos == 1) ? farRpm : nearRpm;
    }

    public double getCurrentRpmEstimate() {
        double ticksPerSec = motor.getVelocity();
        return ticksPerSecToRpm(ticksPerSec);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void stop() {
        isOn = false;
        motor.setPower(0.0);
        // No PID state to reset; motor's internal controller just sees target velocity 0
    }

    // ===== PIDF GAIN ACCESSORS (for tuning in TeleOp) =====

    public double getKp() { return kP; }
    public double getKi() { return kI; }
    public double getKd() { return kD; }
    public double getKf() { return kF; }

    public void adjustKp(double delta) {
        kP = Math.max(0.0, kP + delta);
        applyPidfToMotor();
    }

    public void adjustKi(double delta) {
        kI = Math.max(0.0, kI + delta);
        applyPidfToMotor();
    }

    public void adjustKd(double delta) {
        kD = Math.max(0.0, kD + delta);
        applyPidfToMotor();
    }

    public void adjustKf(double delta) {
        kF = Math.max(0.0, kF + delta);
        applyPidfToMotor();
    }

    public void setKp(double value) { kP = Math.max(0.0, value); applyPidfToMotor(); }
    public void setKi(double value) { kI = Math.max(0.0, value); applyPidfToMotor(); }
    public void setKd(double value) { kD = Math.max(0.0, value); applyPidfToMotor(); }
    public void setKf(double value) { kF = Math.max(0.0, value); applyPidfToMotor(); }
}

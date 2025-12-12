package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;

public class ShooterSubsystem {

    // ===== MOTOR CONSTANTS =====
    // goBILDA 6000 RPM motor
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;
    private static final double MAX_RPM = PHYSICAL_MAX_RPM;

    // ===== HOOD SERVO CONSTANTS =====
    // Tuned from your servo test (adjust in practice):
    //  0.85 -> ~35 degrees (near)
    //  0.92 -> ~60 degrees (far-ish)
    private static final double HOOD_NEAR_POS = 0.85;
    private static final double HOOD_FAR_POS  = 0.92;

    // ===== HELPERS =====
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    // ===== HARDWARE =====
    private final DcMotorEx motor;
    private final Servo hoodServo;
    private LightSubsystem light;

    // ===== STATE =====
    private boolean isOn = false;   // shooter commanded on/off
    private int fieldPos = 0;       // 0 = nearfield, 1 = far

    private double nearRpm = 2700.0;   // tune these on field
    private double farRpm  = 3300.0;

    // edge-detection for RPM buttons
    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== PID STATE =====
    // Start with these and tune
    private double kP = 0.0008;
    private double kI = 0.0004;
    private double kD = 0.0002;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private boolean pidInitialized = false;

    private final ElapsedTime pidTimer = new ElapsedTime();

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        light = new LightSubsystem(hardwareMap);


        // We are doing our own velocity control, so avoid built-in velocity PID
        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Start NEAR field by default
        fieldPos = 0;
        hoodServo.setPosition(HOOD_NEAR_POS);

        pidTimer.reset();
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
            runPidStep(targetRpm);
            if (lastError < 200 || -lastError < 200) {
                light.setColor(3);
            }else{
                light.setColor(2);
            }
        } else {
            stop();
            light.setColor(1);
        }
    }

    // ==== PID UPDATE ====
    private void runPidStep(double targetRpm) {
        double now = pidTimer.seconds();

        if (!pidInitialized) {
            lastTime = now;
            lastError = 0.0;
            integral = 0.0;
            pidInitialized = true;
        }

        double dt = now - lastTime;
        if (dt <= 0) return;

        double currentRpm = getCurrentRpmEstimate();
        double error = targetRpm - currentRpm;

        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double pid = kP * error + kI * integral + kD * derivative;

        // Simple feedforward: target fraction of max rpm
        double ff = targetRpm / MAX_RPM;

        double power = ff + pid;
        power = Range.clip(power, 0.0, 1.0);

        motor.setPower(power);

        lastError = error;
        lastTime = now;
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

        integral = 0.0;
        lastError = 0.0;
        pidInitialized = false;
    }

    // ===== PID GAIN ACCESSORS (for tuning in TeleOp) =====

    public double getKp() { return kP; }
    public double getKi() { return kI; }
    public double getKd() { return kD; }

    public void adjustKp(double delta) {
        kP = Math.max(0.0, kP + delta);
    }

    public void adjustKi(double delta) {
        kI = Math.max(0.0, kI + delta);
    }

    public void adjustKd(double delta) {
        kD = Math.max(0.0, kD + delta);
    }

    // Optional helpers for setting absolute values if you want
    public void setKp(double value) { kP = Math.max(0.0, value); }
    public void setKi(double value) { kI = Math.max(0.0, value); }
    public void setKd(double value) { kD = Math.max(0.0, value); }
}

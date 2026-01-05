package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.util.PIDFController;

public class ShooterSubsystemFF {

    // ===== MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;      // goBILDA 6000 motor encoder
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    private static final double MAX_TICKS_PER_SEC = PHYSICAL_MAX_RPM * TICKS_PER_REV / 60.0;

    // ===== HOOD SERVO CONSTANTS =====
    private static final double HOOD_NEAR_POS = 0.92;
    private static final double HOOD_FAR_POS  = 0.85;

    // ===== HARDWARE =====
    private final DcMotorEx motor;
    private final Servo hoodServo;
    private final LightSubsystem light;

    // ===== CONTROLLER =====
    private final org.firstinspires.ftc.teamcode.subsystems.util.PIDFController controller;

    // ===== STATE =====
    private boolean isOn = false;
    private int fieldPos = 0; // 0 near, 1 far

    private double nearRpm = 3050.0;
    private double farRpm  = 3700.0;

    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== TUNABLES (what you’ll tune) =====
    // Units:
    //  - targetVel is ticks/sec
    //  - kV is power per (ticks/sec)  (~ 1/MAX_TICKS_PER_SEC as a starting point)
    //  - kS is static power offset (sign-applied)
    //  - P/I are “power per ticks/sec error”
    private double kP = 0.00010;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kV = 1.0 / MAX_TICKS_PER_SEC; // good first guess
    private double kS = 0.02;                    // typical small-ish start

    // debug
    private double lastTargetTps = 0.0;
    private double lastVelTps = 0.0;
    private double lastPowerCmd = 0.0;
    private double lastFF = 0.0;

    public ShooterSubsystemFF(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        light = new LightSubsystem(hardwareMap);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // We are doing our own velocity loop -> run open-loop power
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(kP, kI, kD, 0.0);

        fieldPos = 0;
        hoodServo.setPosition(HOOD_NEAR_POS);
    }

    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        // === FIELD POS / HOOD ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;
        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;
            hoodServo.setPosition(fieldPos == 1 ? HOOD_FAR_POS : HOOD_NEAR_POS);
        }

        // === RPM ADJUST (edge) ===
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) farRpm = Math.min(PHYSICAL_MAX_RPM, farRpm + RPM_STEP);
            else               nearRpm = Math.min(PHYSICAL_MAX_RPM, nearRpm + RPM_STEP);
        }
        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) farRpm = Math.max(0.0, farRpm - RPM_STEP);
            else               nearRpm = Math.max(0.0, nearRpm - RPM_STEP);
        }
        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY COMMAND ===
        isOn = shooterOnCommand;

        double targetRpm = getTargetRpm();
        double targetTps = (isOn && targetRpm > 0) ? rpmToTicksPerSec(targetRpm) : 0.0;

        double velTps = motor.getVelocity(); // still works in RUN_WITHOUT_ENCODER
        double errorTps = targetTps - velTps;

        lastTargetTps = targetTps;
        lastVelTps = velTps;

        if (targetTps <= 1.0) {
            // OFF
            motor.setPower(0.0);
            lastPowerCmd = 0.0;
            lastFF = 0.0;
            light.setColor(1);
            return;
        }

        // Feedforward: kV*target + kS*sign(target)
        double ff = kV * targetTps + kS * Math.signum(targetTps);
        lastFF = ff;

        // Push params into controller each loop (so tuning updates instantly)
        controller.setPIDF(kP, kI, kD, ff);

        // Controller output is power
        double power = controller.calculate(errorTps);
        power = Range.clip(power, -1.0, 1.0);

        motor.setPower(power);
        lastPowerCmd = power;

        // Light logic based on RPM error (so it matches your intuition)
        double curRpm = ticksPerSecToRpm(velTps);
        double errRpm = getTargetRpm() - curRpm;

        if (Math.abs(errRpm) < 200) light.setColor(3);
        else light.setColor(2);
    }

    // ===== GETTERS =====
    public boolean isOn() { return isOn; }
    public int getFieldPos() { return fieldPos; }
    public double getNearRpm() { return nearRpm; }
    public double getFarRpm() { return farRpm; }
    public double getTargetRpm() { return (fieldPos == 1) ? farRpm : nearRpm; }

    public double getTargetTps() { return lastTargetTps; }
    public double getVelocityTps() { return lastVelTps; }
    public double getVelocityRpm() { return ticksPerSecToRpm(lastVelTps); }
    public double getPowerCmd() { return lastPowerCmd; }
    public double getFF() { return lastFF; }

    public double getKp() { return kP; }
    public double getKi() { return kI; }
    public double getKd() { return kD; }
    public double getKv() { return kV; }
    public double getKs() { return kS; }

    public void adjustKp(double d) { kP = Math.max(0.0, kP + d); }
    public void adjustKi(double d) { kI = Math.max(0.0, kI + d); }
    public void adjustKd(double d) { kD = Math.max(0.0, kD + d); }
    public void adjustKv(double d) { kV = Math.max(0.0, kV + d); }
    public void adjustKs(double d) { kS = kS + d; } // allow negative if you want to simulate sag

    public void stop() {
        isOn = false;
        motor.setPower(0.0);
        light.setColor(1);
    }
}

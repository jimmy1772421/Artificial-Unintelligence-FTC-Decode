package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.util.PIDFController;

@Configurable
public class ShooterSubsystemFF_dualMotor {

    // ===== ENCODER / MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;      // goBILDA 6000 motor encoder (ticks/rev of motor output shaft)
    private static final double PHYSICAL_MAX_MOTOR_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    // ===== GEAR RATIO (motor gear : flywheel gear) =====
    // If motor gear is 4T and flywheel gear is 6T:
    // flywheel_rpm = motor_rpm * (4/6)
    private static final double MOTOR_TO_FLYWHEEL = 4.0 / 6.0;
    private static final double FLYWHEEL_TO_MOTOR = 1.0 / MOTOR_TO_FLYWHEEL;

    private static final double PHYSICAL_MAX_FLYWHEEL_RPM = PHYSICAL_MAX_MOTOR_RPM * MOTOR_TO_FLYWHEEL;

    private static final double MAX_MOTOR_TICKS_PER_SEC = PHYSICAL_MAX_MOTOR_RPM * TICKS_PER_REV / 60.0;

    // ===== HOOD SERVO CONSTANTS =====
    private static final double HOOD_NEAR_POS = 0.92;
    private static final double HOOD_FAR_POS  = 0.92;

    // =========================
    // PANELS TUNABLES (STATIC)
    // =========================
    // Treat these as FLYWHEEL RPM setpoints
    public static double TUNE_NEAR_RPM = 2900.0;
    public static double TUNE_FAR_RPM  = 3100.0;

    public static double TUNE_HOOD_NEAR_POS = HOOD_NEAR_POS;
    public static double TUNE_HOOD_FAR_POS  = HOOD_FAR_POS;

    public static double TUNE_kP = 0.01;
    public static double TUNE_kI = 0.0;
    public static double TUNE_kD = 0.0;
    public static double TUNE_kV = 0.00042; // note: with gear reduction, target motor tps is higher for same flywheel rpm
    public static double TUNE_kS = 0.05;

    public static boolean TUNE_FORCE_ON = false;
    public static double  TUNE_FORCE_RPM = 3000.0; // FLYWHEEL rpm while tuning

    // ===== HARDWARE =====
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final Servo hoodServo;
    private final LightSubsystem light;

    // ===== CONTROLLER =====
    private final PIDFController controller;

    // ===== STATE =====
    private boolean isOn = false;
    private int fieldPos = 0; // 0 near, 1 far

    // Treat as FLYWHEEL rpm targets
    private double nearRpm = 3050.0;
    private double farRpm  = 3700.0;

    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== TUNABLES (INSTANCE COPIES) =====
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kV = 0.0;
    private double kS = 0.0;

    // debug (motor-domain tps + flywheel-domain rpm)
    private double lastTargetMotorTps = 0.0;
    private double lastVelMotorTps = 0.0;
    private double lastPowerCmd = 0.0;
    private double lastFF = 0.0;

    private double lastVelMotor1Tps = 0.0;
    private double lastVelMotor2Tps = 0.0;

    public ShooterSubsystemFF_dualMotor(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor_one");
        motor2 = hardwareMap.get(DcMotorEx.class, "turretMotor"); // <-- add this in config
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        light = new LightSubsystem(hardwareMap);

        initMotor(motor1);
        initMotor(motor2);

        // If the second motor is mirrored and spins the wrong way, flip it:
        // motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Sync instance from Panels tunables once on init
        syncFromTunables();

        controller = new PIDFController(kP, kI, kD, 0.0);

        fieldPos = 0;
        hoodServo.setPosition(TUNE_HOOD_NEAR_POS);
    }

    private void initMotor(DcMotorEx m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void syncFromTunables() {
        nearRpm = TUNE_NEAR_RPM;
        farRpm  = TUNE_FAR_RPM;

        kP = TUNE_kP;
        kI = TUNE_kI;
        kD = TUNE_kD;
        kV = TUNE_kV;
        kS = TUNE_kS;
    }

    private void syncToTunables() {
        TUNE_NEAR_RPM = nearRpm;
        TUNE_FAR_RPM  = farRpm;

        TUNE_kP = kP;
        TUNE_kI = kI;
        TUNE_kD = kD;
        TUNE_kV = kV;
        TUNE_kS = kS;
    }

    private static double ticksPerSecToMotorRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private static double motorRpmToTicksPerSec(double motorRpm) {
        return motorRpm * TICKS_PER_REV / 60.0;
    }

    private static double motorRpmToFlywheelRpm(double motorRpm) {
        return motorRpm * MOTOR_TO_FLYWHEEL;
    }

    private static double flywheelRpmToMotorRpm(double flywheelRpm) {
        return flywheelRpm * FLYWHEEL_TO_MOTOR;
    }

    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        // Always pull latest Panels values at start of loop
        syncFromTunables();

        // === FIELD POS / HOOD ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;
        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;
            hoodServo.setPosition(fieldPos == 1 ? TUNE_HOOD_FAR_POS : TUNE_HOOD_NEAR_POS);
        }

        // === RPM ADJUST (edge) ===
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) farRpm  = Math.min(PHYSICAL_MAX_FLYWHEEL_RPM, farRpm + RPM_STEP);
            else               nearRpm = Math.min(PHYSICAL_MAX_FLYWHEEL_RPM, nearRpm + RPM_STEP);
            syncToTunables();
        }
        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) farRpm  = Math.max(0.0, farRpm - RPM_STEP);
            else               nearRpm = Math.max(0.0, nearRpm - RPM_STEP);
            syncToTunables();
        }
        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY COMMAND ===
        boolean on = shooterOnCommand || TUNE_FORCE_ON;
        isOn = on;

        double targetFlywheelRpm = TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm(); // flywheel rpm
        double targetMotorRpm = (isOn && targetFlywheelRpm > 0) ? flywheelRpmToMotorRpm(targetFlywheelRpm) : 0.0;
        double targetMotorTps = (targetMotorRpm > 0) ? motorRpmToTicksPerSec(targetMotorRpm) : 0.0;

        // Read both motor velocities and average (motor-domain tps)
        double v1 = motor1.getVelocity();
        double v2 = motor2.getVelocity();
        lastVelMotor1Tps = v1;
        lastVelMotor2Tps = v2;

        double velMotorTps = 0.5 * (v1 + v2);

        double errorTps = targetMotorTps - velMotorTps;

        lastTargetMotorTps = targetMotorTps;
        lastVelMotorTps = velMotorTps;

        if (targetMotorTps <= 1.0) {
            // OFF
            motor1.setPower(0.0);
            motor2.setPower(0.0);
            lastPowerCmd = 0.0;
            lastFF = 0.0;
            light.setColor(1);
            return;
        }

        // Feedforward uses MOTOR-domain target velocity (ticks/sec)
        double ff = (kV * targetMotorTps) + kS;
        lastFF = ff;

        controller.setPIDF(kP, kI, kD, ff);

        double power = controller.calculate(errorTps);
        power = Range.clip(power, -1.0, 1.0);

        motor1.setPower(power);
        motor2.setPower(power);
        lastPowerCmd = power;

        // Light logic based on FLYWHEEL rpm error
        double curMotorRpm = ticksPerSecToMotorRpm(velMotorTps);
        double curFlywheelRpm = motorRpmToFlywheelRpm(curMotorRpm);
        double errFlywheelRpm = targetFlywheelRpm - curFlywheelRpm;

        if (Math.abs(errFlywheelRpm) < 150) light.setColor(3);
        else light.setColor(2);
    }

    // ===== GETTERS =====
    public boolean isOn() { return isOn; }
    public int getFieldPos() { return fieldPos; }

    // These are FLYWHEEL rpm targets
    public double getNearRpm() { return nearRpm; }
    public double getFarRpm() { return farRpm; }
    public double getTargetRpm() { return (fieldPos == 1) ? farRpm : nearRpm; }

    // Motor-domain (for debugging controller)
    public double getTargetMotorTps() { return lastTargetMotorTps; }
    public double getVelocityMotorTps() { return lastVelMotorTps; }
    public double getMotor1VelocityTps() { return lastVelMotor1Tps; }
    public double getMotor2VelocityTps() { return lastVelMotor2Tps; }

    // Flywheel-domain readouts (what you actually care about)
    public double getVelocityRpm() {
        double motorRpm = ticksPerSecToMotorRpm(lastVelMotorTps);
        return motorRpmToFlywheelRpm(motorRpm);
    }

    public double getPowerCmd() { return lastPowerCmd; }
    public double getFF() { return lastFF; }

    public void stop() {
        isOn = false;
        motor1.setPower(0.0);
        motor2.setPower(0.0);
        light.setColor(1);
    }

    public double getCurrentRpmEstimate() {
        // returns FLYWHEEL rpm estimate
        double motorTps = 0.5 * (motor1.getVelocity() + motor2.getVelocity());
        double motorRpm = ticksPerSecToMotorRpm(motorTps);
        return motorRpmToFlywheelRpm(motorRpm);
    }

    // Returns the SAME target the controller is actually using (flywheel RPM)
    public double getActiveTargetRpm() {
        return TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm();
    }

    // Flywheel RPM error = target - current
    public double getErrorRpm() {
        return getActiveTargetRpm() - getCurrentRpmEstimate();
    }

}

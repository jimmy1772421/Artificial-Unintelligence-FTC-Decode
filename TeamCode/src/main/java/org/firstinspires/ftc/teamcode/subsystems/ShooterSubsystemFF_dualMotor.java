package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.util.PIDFController;

@Configurable
public class ShooterSubsystemFF_dualMotor {

    // ===== ENCODER / MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_MOTOR_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    // ===== GEAR RATIO (motor gear : flywheel gear) =====
    private static final double MOTOR_TO_FLYWHEEL = 4.0 / 6.0;
    private static final double FLYWHEEL_TO_MOTOR = 1.0 / MOTOR_TO_FLYWHEEL;

    private static final double PHYSICAL_MAX_FLYWHEEL_RPM = PHYSICAL_MAX_MOTOR_RPM * MOTOR_TO_FLYWHEEL;

    // ===== HOOD SERVO CONSTANTS =====
    private static final double HOOD_NEAR_POS = 0.92;
    private static final double HOOD_FAR_POS  = 0.92;

    // =========================
    // PANELS TUNABLES (STATIC)
    // =========================
    public static double TUNE_NEAR_RPM = 2900.0;
    public static double TUNE_FAR_RPM  = 3100.0;

    public static double TUNE_HOOD_NEAR_POS = HOOD_NEAR_POS;
    public static double TUNE_HOOD_FAR_POS  = HOOD_FAR_POS;

    public static double TUNE_kP = 0.01;
    public static double TUNE_kI = 0.0;
    public static double TUNE_kD = 0.0;
    public static double TUNE_kV = 0.00042;
    public static double TUNE_kS = 0.05;

    public static boolean TUNE_FORCE_ON = false;
    public static double  TUNE_FORCE_RPM = 3000.0;

    // ===== HARDWARE =====
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx hoodServo;
    private final LightSubsystem light;

    // ===== CONTROLLER =====
    private final PIDFController controller;

    // ===== STATE =====
    private boolean isOn = false;
    private int fieldPos = 0; // 0 near, 1 far

    private double nearRpm = 3050.0;
    private double farRpm  = 3700.0;

    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== TUNABLES (INSTANCE COPIES) =====
    private double kP = 0.0, kI = 0.0, kD = 0.0, kV = 0.0, kS = 0.0;

    // debug
    private double lastTargetMotorTps = 0.0;
    private double lastVelMotorTps = 0.0;
    private double lastPowerCmd = 0.0;
    private double lastFF = 0.0;

    private double lastVelMotor1Tps = 0.0;
    private double lastVelMotor2Tps = 0.0;

    public ShooterSubsystemFF_dualMotor(HardwareMap hardwareMap) {
        // If you want SolversLib to “know” CPR/RPM, use the (cpr, rpm) constructor
        motor1 = new MotorEx(hardwareMap, "motor_one", TICKS_PER_REV, PHYSICAL_MAX_MOTOR_RPM);
        motor2 = new MotorEx(hardwareMap, "motor_two", TICKS_PER_REV, PHYSICAL_MAX_MOTOR_RPM);

        hoodServo = new ServoEx(hardwareMap, "hoodServo");

        light = new LightSubsystem(hardwareMap);

        initMotor(motor1);
        initMotor(motor2);

        // In SolversLib, direction is typically handled via inversion:
        motor1.setInverted(true);   // equivalent to REVERSE
        motor2.setInverted(false);  // equivalent to FORWARD

        // Optional write-caching: only helps if you spam nearly-identical outputs
        // motor1.setCachingTolerance(0.0005);
        // motor2.setCachingTolerance(0.0005);
        // hoodServo.setCachingTolerance(0.001);

        syncFromTunables();
        controller = new PIDFController(kP, kI, kD, 0.0);

        fieldPos = 0;
        hoodServo.set(TUNE_HOOD_NEAR_POS); // <-- FIX: use set(), not setPosition()
    }

    private void initMotor(MotorEx m) {
        m.stopAndResetEncoder();
        m.setRunMode(Motor.RunMode.RawPower);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
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

    private static double ticksPerSecToMotorRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
    private static double motorRpmToTicksPerSec(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private static double motorRpmToFlywheelRpm(double rpm) { return rpm * MOTOR_TO_FLYWHEEL; }
    private static double flywheelRpmToMotorRpm(double rpm) { return rpm * FLYWHEEL_TO_MOTOR; }

    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        syncFromTunables();

        // === FIELD POS / HOOD ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;
        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;
            hoodServo.set(fieldPos == 1 ? TUNE_HOOD_FAR_POS : TUNE_HOOD_NEAR_POS); // <-- set()
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


        double targetFlywheelRpm = TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm();
        double targetMotorRpm = (isOn && targetFlywheelRpm > 0) ? flywheelRpmToMotorRpm(targetFlywheelRpm) : 0.0;
        double targetMotorTps = (targetMotorRpm > 0) ? motorRpmToTicksPerSec(targetMotorRpm) : 0.0;

        // Read both motor velocities (ticks/sec)
        double v1 = motor1.getVelocity();
        double v2 = motor2.getVelocity();
        lastVelMotor1Tps = v1;
        lastVelMotor2Tps = v2;

        double velMotorTps = 0.5 * (v1 + v2);
        double errorTps = targetMotorTps - velMotorTps;

        lastTargetMotorTps = targetMotorTps;
        lastVelMotorTps = velMotorTps;

        if (targetMotorTps <= 1.0) {
            motor1.set(0.0);
            motor2.set(0.0);
            lastPowerCmd = 0.0;
            lastFF = 0.0;
            light.setColor(1);
            return;
        }

        double ff = (kV * targetMotorTps) + kS;
        lastFF = ff;

        controller.setPIDF(kP, kI, kD, ff);

        double power = controller.calculate(errorTps);
        power = Range.clip(power, -1.0, 1.0);

        motor1.set(power);
        motor2.set(power);
        lastPowerCmd = power;

        // Light logic based on flywheel rpm error
        double curMotorRpm = ticksPerSecToMotorRpm(velMotorTps);
        double curFlywheelRpm = motorRpmToFlywheelRpm(curMotorRpm);
        double errFlywheelRpm = targetFlywheelRpm - curFlywheelRpm;

        light.setColor(Math.abs(errFlywheelRpm) < 150 ? 3 : 2);
    }

    // ===== GETTERS =====
    public boolean isOn() { return isOn; }
    public int getFieldPos() { return fieldPos; }
    public double getTargetRpm() { return (fieldPos == 1) ? farRpm : nearRpm; }

    public double getTargetMotorTps() { return lastTargetMotorTps; }
    public double getVelocityMotorTps() { return lastVelMotorTps; }
    public double getMotor1VelocityTps() { return lastVelMotor1Tps; }
    public double getMotor2VelocityTps() { return lastVelMotor2Tps; }

    public double getVelocityRpm() {
        double motorRpm = ticksPerSecToMotorRpm(lastVelMotorTps);
        return motorRpmToFlywheelRpm(motorRpm);
    }

    public double getPowerCmd() { return lastPowerCmd; }
    public double getFF() { return lastFF; }

    public void stop() {
        isOn = false;
        motor1.set(0.0);
        motor2.set(0.0);
        light.setColor(1);
    }
}

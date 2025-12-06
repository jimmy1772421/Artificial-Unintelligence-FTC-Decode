package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
public class TurretSubsystem {
    // goBILDA 435 RPM motor
    private static final double TICKS_PER_REV = 383.6;
    private static final double PHYSICAL_MAX_RPM = 435.0;
    private static final double RPM_STEP = 15.0;
    private static final double MOTOR_REV_PER_TURRET_REV = 290.0/60.0;
    private static final double TICKS_PER_TURRET_DEG = TICKS_PER_REV * MOTOR_REV_PER_TURRET_REV / 360.0;//for abs encoder too


    //angle limits
    private static final double MIN_ANGLE_DEG = -120.0;
    private static final double MAX_ANGLE_DEG = 120.0;

    //proportional gain for position control
    private static final double kP = 0.01;
    private static final double MAX_AUTO_POWER = 0.5;

    private final DcMotorEx turretMotor;
    private final AnalogInput turretAbs;  // analog absolute encoder

    //ABS encoder calib
    private static final double ZERO_ABS_DEG = 123.4;  // <-- replace with your measured value
    // Flip sign if abs encoder runs backwards
    private static final double ANGLE_DIRECTION = 1.0;

    // Offset so that ticks → real turret angle:
    // turretAngleDeg = angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG)
    private double angleOffsetDeg = 0.0;

    private double maxVoltage;

    //control state
    private boolean positionMode = false; //false = manual, true = auto angle
    private double targetAngleDeg = 0.0; //desired turret angle
    private double manualPower = 0.0; // joystick power when in manual

    public TurretSubsystem (HardwareMap hardwareMap){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretAbs = hardwareMap.get(AnalogInput.class, "turretABS");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use ELC encoder not the on motor one

        maxVoltage = turretAbs.getMaxVoltage();

        // Use the absolute encoder ONCE to line up motor ticks with real turret angle
        computeAngleOffsetFromAbs();

    }


    // Internal helpers
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    // --- ABS ENCODER HELPERS ---

    /** Raw abs angle 0..360 from the analog encoder. */
    private double getAbsAngle0to360() {
        double v = turretAbs.getVoltage();
        double frac = v / maxVoltage;  // 0..1
        return frac * 360.0;
    }

    /** Wrap angle to [-180, 180). */
    private double wrapDeg(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

    /**
     * Compute angleOffsetDeg so that:
     *   abs angle (from analog) == angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
     * Call once at startup.
     */
    private void computeAngleOffsetFromAbs() {
        double rawAbs = getAbsAngle0to360();                 // 0..360
        double absRelative = rawAbs - ZERO_ABS_DEG;          // relative to your mechanical zero
        absRelative = wrapDeg(absRelative) * ANGLE_DIRECTION;

        int ticks = turretMotor.getCurrentPosition();

        // Solve: absRelative = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
        angleOffsetDeg = absRelative - (ticks / TICKS_PER_TURRET_DEG);
    }

    // --- ANGLE APIs ---

    /** Current turret angle in degrees, using motor ticks + calibrated offset. */
    public double getCurrentAngleDeg() {
        int ticks = turretMotor.getCurrentPosition();
        return angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    // --- CONTROL MODES ---

    /** Manual mode: power from joystick. */
    public void setManualPower(double power) {
        positionMode = false;
        power = Range.clip(power, -1.0, 1.0);

        // Make sure we're not stuck in RUN_TO_POSITION
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        turretMotor.setPower(power);
    }

    /** Position mode: go to a specific turret angle using RUN_TO_POSITION. */
    public void goToAngle(double angleDeg) {
        angleDeg = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        targetAngleDeg = angleDeg;
        positionMode = true;

        // Convert desired angle to motor ticks:
        // angleDeg = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
        // → ticks = (angleDeg - angleOffsetDeg) * TICKS_PER_TURRET_DEG
        int targetTicks = (int) Math.round((angleDeg - angleOffsetDeg) * TICKS_PER_TURRET_DEG);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a reasonable power cap; PID is handled by the built-in controller
        turretMotor.setPower(0.4);
    }

    /** Call this every loop just for telemetry / mode management. */
    public void update() {
        // You *can* add logic here if you want to auto-switch back to manual
        // after reaching target, but it's not strictly required.
    }

}

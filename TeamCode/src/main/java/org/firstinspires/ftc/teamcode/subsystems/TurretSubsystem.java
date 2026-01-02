package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {
    // ===== MOTOR / GEAR CONSTANTS =====
    // goBILDA 435 RPM motor integrated encoder
    private static final double TICKS_PER_MOTOR_REV = 383.6;

    // 290T turret gear, 60T motor pulley → motor spins 290/60 times per turret rev
    private static final double MOTOR_REV_PER_TURRET_REV = 290.0 / 60.0;
    private static final double TICKS_PER_TURRET_REV =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV;
    private static final double TICKS_PER_TURRET_DEG = TICKS_PER_TURRET_REV / 360.0;

    // Angle limits (turret frame)
    private static final double MIN_ANGLE_DEG = -200.0;
    private static final double MAX_ANGLE_DEG =  200.0;

    // When auto-positioning
    private static final double MAX_AUTO_POWER = 0.5;

    // How far tick-angle is allowed to drift from abs before we re-sync
    private static final double RESYNC_THRESHOLD_DEG = 5.0;

    // ===== HARDWARE =====
    private final DcMotorEx turretMotor;
    private final AnalogInput turretAbs;  // analog absolute encoder on turret

    // Abs encoder calibration:
    // raw abs angle (0..360) when turret is at "0°" (centered / your chosen zero)
    private static final double ZERO_ABS_DEG = 123.4; // <-- calibrate this!!!!!!
    // Flip this if abs angle increases opposite your "positive turret" direction
    private static final double ANGLE_DIRECTION = 1.0; // or -1.0

    // Relationship between motor ticks and turret angle:
    // turretAngleDeg = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
    private double angleOffsetDeg = 0.0;

    // Control state
    private boolean positionMode = false;   // false = manual, true = RUN_TO_POSITION
    private double targetAngleDeg = 0.0;

    private double maxVoltage;

    private double lastAbsRaw = 0.0;
    private double absUnwrapped = 0.0;
    private boolean absInit = false;


    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretAbs   = hardwareMap.get(AnalogInput.class, "turretABS");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // We'll switch modes as needed, but RUN_USING_ENCODER is a safe default
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        maxVoltage = turretAbs.getMaxVoltage();

        // One-time alignment of motor ticks with the absolute encoder
        computeAngleOffsetFromAbs();
    }

    // ===== ABS ENCODER HELPERS =====

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
     * Turret angle directly from the abs encoder (no motor ticks).
     * Returns something like [-180, 180) in your turret frame.
     */
    private double getAbsTurretAngleDeg() {
        double rawAbs = getAbsAngle0to360();      // 0..360
        double rel = rawAbs - ZERO_ABS_DEG;       // 0 when turret at mechanical zero
        rel = wrapDeg(rel) * ANGLE_DIRECTION;     // apply direction & wrap
        return rel;
    }

    /**
     * Compute angleOffsetDeg so that:
     *   absAngle == angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
     * Call at startup and any time we decide the tick-angle has drifted.
     */
    private void computeAngleOffsetFromAbs() {
        double absAngle = getAbsTurretAngleDegContinuous();   // what turret really is
        int ticks = turretMotor.getCurrentPosition();
        double tickAngle = ticks / TICKS_PER_TURRET_DEG;
        angleOffsetDeg = absAngle - tickAngle;
    }

    /**
     * If tick-based angle drifts too far from abs angle, re-align them.
     * You can call this in update() every loop.
     */
    private void resyncFromAbsIfNeeded() {
        double absAngle  = getAbsTurretAngleDegContinuous();
        double tickAngle = getCurrentAngleDeg();
        double diff = wrapDeg(absAngle - tickAngle);

        if (Math.abs(diff) > RESYNC_THRESHOLD_DEG) {
            // Re-calc offset so ticks match abs again
            computeAngleOffsetFromAbs();
        }
    }

    // ===== ANGLE / TICKS HELPERS =====

    /** Current turret angle (deg) from motor ticks + calibrated offset. */
    public double getCurrentAngleDeg() {
        int ticks = turretMotor.getCurrentPosition();
        return angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG);
    }

    private double getAbsTurretAngleDegContinuous() {
        double raw = getAbsAngle0to360(); // 0..360

        if (!absInit) {
            absInit = true;
            lastAbsRaw = raw;
            absUnwrapped = raw;
        } else {
            double delta = raw - lastAbsRaw;
            // unwrap across 0/360 boundary
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            absUnwrapped += delta;
            lastAbsRaw = raw;
        }

        // now absUnwrapped is continuous degrees, still in "raw frame"
        double rel = (absUnwrapped - ZERO_ABS_DEG) * ANGLE_DIRECTION;
        return rel;
    }


    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    /**
     * Convert desired turret angle to the CLOSEST tick target,
     * so the motor takes the shortest path (handles wrap).
     */
    private int shortestTicksForAngle(double angleDeg) {
        double clippedAngle = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        // Base tick position for this angle
        double desiredTicksDouble = (clippedAngle - angleOffsetDeg) * TICKS_PER_TURRET_DEG;
        int baseTicks = (int) Math.round(desiredTicksDouble);

        int currentTicks = turretMotor.getCurrentPosition();
        int ticksPerRev = (int) Math.round(TICKS_PER_TURRET_REV);

        int diff = baseTicks - currentTicks;
        int k = (int) Math.round((double) diff / ticksPerRev);

        // Shift by whole turret revs so we get the nearest equivalent position
        int bestTicks = baseTicks - k * ticksPerRev;
        return bestTicks;
    }

    // ===== CONTROL MODES =====

    /** Manual mode: power from joystick (no angle limit unless you add it here). */
    public void setManualPower(double power) {
        positionMode = false;

        double current = getCurrentAngleDeg();
        double marginDeg = 5.0;  // buffer before you actually hit the hard limit

        // If pushing positive (CCW?) and we're already near +limit, kill power
        if (power > 0 && current >= MAX_ANGLE_DEG - marginDeg) {
            power = 0.0;
        }
        // If pushing negative (CW?) and near -limit, kill power
        else if (power < 0 && current <= MIN_ANGLE_DEG + marginDeg) {
            power = 0.0;
        }

        power = Range.clip(power, -1.0, 1.0);

        // Make sure we aren’t stuck in RUN_TO_POSITION
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER
                && turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        turretMotor.setPower(power);
    }


    /** Position mode: go to a specific turret angle using RUN_TO_POSITION. */
    public void goToAngle(double angleDeg) {
        targetAngleDeg = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        positionMode = true;

        int targetTicks = shortestTicksForAngle(targetAngleDeg);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_AUTO_POWER);
    }


    /**
     * Call this every loop from TeleOp / Auto.
     * Right now it just keeps the encoder aligned to the abs angle.
     */
    public void update() {
        // Periodically re-sync ticks to abs if we've drifted
        resyncFromAbsIfNeeded();

        // You could also add logic here to detect when RUN_TO_POSITION
        // has finished and automatically switch back to manual mode.
    }


}

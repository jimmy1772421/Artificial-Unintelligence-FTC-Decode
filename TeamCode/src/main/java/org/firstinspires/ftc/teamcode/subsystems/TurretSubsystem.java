package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import com.pedropathing.geometry.Pose;


public class TurretSubsystem {
    // =========================
    // Motor / gear constants
    // =========================
    // goBILDA 435 RPM motor integrated encoder (output shaft ticks/rev)
    private static final double TICKS_PER_MOTOR_REV = 383.6;

    // 290T turret gear, 60T motor pulley → motor spins 290/60 times per turret rev
    private static final double MOTOR_REV_PER_TURRET_REV = 290.0 / 60.0;

    private static final double TICKS_PER_TURRET_REV =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV;
    private static final double TICKS_PER_TURRET_DEG = TICKS_PER_TURRET_REV / 360.0;

    // Turret mechanical window (your chosen turret frame)
    private static final double MIN_ANGLE_DEG = -200.0;
    private static final double MAX_ANGLE_DEG =  200.0;

    private static final double MAX_AUTO_POWER = 0.5;

    // Resync only when stopped; only if we drift more than this
    private static final double RESYNC_THRESHOLD_DEG = 5.0;
    private static final double STOP_VELOCITY_TPS = 80.0; // ticks/sec; tune 50~200

    // =========================
    // Hardware
    // =========================
    private final DcMotorEx turretMotor;
    private final AnalogInput turretAbs;  // analog absolute encoder

    // Abs encoder calibration:
    // raw abs angle (0..360) when turret is at "0°" (centered / your chosen zero)
    private static final double ZERO_ABS_DEG = 190.3;   // calibrate
    // Flip if abs increases opposite your positive turret direction
    private static final double ANGLE_DIRECTION = 1.0;  // or -1.0

    // ticks -> turret angle:
    // turretAngleDeg = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
    private double angleOffsetDeg = 0.0;

    // Control state
    private boolean positionMode = false;
    private double targetAngleDeg = 0.0;

    private double maxVoltage;

    // =========================
    // ABS continuous unwrap state
    // =========================
    private boolean absInit = false;
    private double lastAbs0to360 = 0.0;
    private double absUnwrapped0to360 = 0.0; // continuous raw (can exceed 0..360)

    private static final double TURRET_MOUNT_OFFSET_DEG = 0.0;

    // Small deadband so we don’t spam new targets when dx/dy are tiny
    private static final double FACE_TARGET_MIN_DIST_IN = 0.5;

    /**
     * Static carry-over within the same RC app session (Auto -> TeleOp, etc.)
     * This prevents the ABS "branch flip" when you restart an OpMode while turret is near the overlap band.
     */
    //public static Double lastKnownTurretAngleDeg = null;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretAbs   = hardwareMap.get(AnalogInput.class, "turretABS");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // IMPORTANT: do NOT reset encoder here if you want continuity across OpModes
        // turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        maxVoltage = turretAbs.getMaxVoltage();

        // Seed ABS unwrapping to the correct 360-branch if we have a saved angle (Auto -> TeleOp)
        seedAbsUnwrapFromSavedAngle();

        // Align motor ticks to ABS once at init
        computeAngleOffsetFromAbs();
    }

    // =========================
    // ABS helpers
    // =========================
    /** Raw abs angle 0..360 from analog encoder. */
    private double getAbsAngle0to360() {
        double v = turretAbs.getVoltage();
        double frac = (maxVoltage <= 1e-6) ? 0.0 : (v / maxVoltage);
        return Range.clip(frac, 0.0, 1.0) * 360.0;
    }

    /** Wrap to [-180, 180). */
    private double wrap180(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

    /** Small-diff wrap for comparing two angles that should be "close". */
    private double wrapSmallDiff(double diff) {
        return wrap180(diff);
    }

    /**
     * Your requested unwrap logic:
     * if last > 300 and now < 60 => wrapped forward
     * if last < 60 and now > 300 => wrapped backward
     */
    private double delta0to360WithWrap(double lastDeg, double nowDeg) {
        if (lastDeg > 300.0 && nowDeg < 60.0) {
            return (360.0 - lastDeg) + nowDeg;      // forward wrap
        } else if (lastDeg < 60.0 && nowDeg > 300.0) {
            return -((360.0 - nowDeg) + lastDeg);   // backward wrap
        } else {
            return nowDeg - lastDeg;
        }
    }

    /**
     * Continuous turret angle from ABS (can exceed +/-180).
     * This is "truth" while the OpMode is running.
     */
    private double getAbsTurretAngleDegContinuous() {
        double now = getAbsAngle0to360();

        if (!absInit) {
            absInit = true;
            lastAbs0to360 = now;
            absUnwrapped0to360 = now;
        } else {
            double d = delta0to360WithWrap(lastAbs0to360, now);
            absUnwrapped0to360 += d;
            lastAbs0to360 = now;
        }

        // convert to turret frame
        return (absUnwrapped0to360 - ZERO_ABS_DEG) * ANGLE_DIRECTION;
    }

    /**
     * If we have a saved turret angle from the previous OpMode, seed absUnwrapped0to360
     * so ABS starts on the correct 360° branch (prevents +190 becoming -170 at TeleOp start).
     */
    private void seedAbsUnwrapFromSavedAngle() {
        double raw = getAbsAngle0to360();

        absInit = true;
        lastAbs0to360 = raw;

        if (PoseStorage.lastTurretAngleDeg == null) {
            // No prior info; start simple (best if you start near forward)
            absUnwrapped0to360 = raw;
            return;
        }

        // We want: (absUnwrapped - ZERO) * dir ~= savedAngle
        // absUnwrapped ~= (savedAngle/dir) + ZERO
        double desiredAbsUnwrapped = (PoseStorage.lastTurretAngleDeg / ANGLE_DIRECTION) + ZERO_ABS_DEG;

        // Choose k so raw + 360k is closest to desiredAbsUnwrapped
        double k = Math.round((desiredAbsUnwrapped - raw) / 360.0);
        absUnwrapped0to360 = raw + 360.0 * k;
    }

    // =========================
    // Tick / angle helpers
    // =========================
    /** Current turret angle estimate from motor ticks + offset. */
    public double getCurrentAngleDeg() {
        int ticks = turretMotor.getCurrentPosition();
        return angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    /** Allows Auto -> TeleOp handoff: set current turret angle reference without resetting encoders. */
    public void setAngleReferenceDeg(double turretAngleDeg) {
        int ticks = turretMotor.getCurrentPosition();
        double tickAngle = ticks / TICKS_PER_TURRET_DEG;
        angleOffsetDeg = turretAngleDeg - tickAngle;

        // also seed ABS unwrap to match this reference (so resync behaves)
        PoseStorage.lastTurretAngleDeg = turretAngleDeg;
        seedAbsUnwrapFromSavedAngle();
    }

    /**
     * Wrap commanded angle into [MIN, MAX] using desired + 360*k equivalents,
     * and choose the one closest to currentAngleDeg (shortest safe motion).
     */
    private double wrapCommandToLimits(double desiredDeg, double currentDeg) {
        int kMin = (int) Math.ceil((MIN_ANGLE_DEG - desiredDeg) / 360.0);
        int kMax = (int) Math.floor((MAX_ANGLE_DEG - desiredDeg) / 360.0);

        if (kMin > kMax) {
            return Range.clip(desiredDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        }

        double best = desiredDeg + 360.0 * kMin;
        double bestDist = Math.abs(best - currentDeg);

        for (int k = kMin + 1; k <= kMax; k++) {
            double cand = desiredDeg + 360.0 * k;
            double dist = Math.abs(cand - currentDeg);
            if (dist < bestDist) {
                best = cand;
                bestDist = dist;
            }
        }
        return best;
    }

    // =========================
    // ABS <-> ticks alignment
    // =========================
    /** Compute offset so that ABS angle == offset + tickAngle. */
    private void computeAngleOffsetFromAbs() {
        double absAngle = getAbsTurretAngleDegContinuous();
        int ticks = turretMotor.getCurrentPosition();
        double tickAngle = ticks / TICKS_PER_TURRET_DEG;
        angleOffsetDeg = absAngle - tickAngle;
    }

    /** Resync only when stopped; avoids branch flips by wrapping only the small difference. */
    private void resyncFromAbsIfNeeded() {
        double absAngle  = getAbsTurretAngleDegContinuous();
        double tickAngle = getCurrentAngleDeg();

        double diff = wrapSmallDiff(absAngle - tickAngle); // should be small if we're on correct branch
        if (Math.abs(diff) > RESYNC_THRESHOLD_DEG) {
            angleOffsetDeg += diff; // gentle correction without jumping targets
        }
    }

    // =========================
    // Control
    // =========================
    /** Manual mode with soft limits. */
    public void setManualPower(double power) {
        positionMode = false;

        double current = getCurrentAngleDeg();
        double marginDeg = 5.0;

        if (power > 0 && current >= MAX_ANGLE_DEG - marginDeg) power = 0.0;
        else if (power < 0 && current <= MIN_ANGLE_DEG + marginDeg) power = 0.0;

        power = Range.clip(power, -1.0, 1.0);

        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER
                && turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        turretMotor.setPower(power);
    }

    /**
     * Position mode: go to a turret angle.
     * - wraps commands like +250 -> -110 (equivalent in window),
     * - uses ABS (truth) to compute error, then converts to ticks delta,
     *   so "goToAngle(0)" is repeatable and doesn’t depend on a perfect offset.
     */
    public void goToAngle(double commandedAngleDeg) {
        positionMode = true;

        double absNow = getAbsTurretAngleDegContinuous();
        targetAngleDeg = wrapCommandToLimits(commandedAngleDeg, absNow);

        // Do NOT wrap this error to [-180,180]; turret window is 400°.
        double errorDeg = targetAngleDeg - absNow;

        int currentTicks = turretMotor.getCurrentPosition();
        int deltaTicks = (int) Math.round(errorDeg * TICKS_PER_TURRET_DEG);
        int targetTicks = currentTicks + deltaTicks;

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_AUTO_POWER);
    }

    // =========================
    // Tracking mode
    // =========================
    private boolean trackEnabled = false;

    public void setTrackEnabled(boolean enabled) { trackEnabled = enabled; }
    public boolean isTrackEnabled() { return trackEnabled; }

    public void trackWithTxDeg(double txDeg) {
        if (!trackEnabled) return;
        if (Double.isNaN(txDeg) || Math.abs(txDeg) < 0.4) return;

        // Camera on turret: tx ~ turret error
        double newTarget = getCurrentAngleDeg() + txDeg; // flip sign if needed
        goToAngle(newTarget);
    }

    // Wrap degrees to [-180, 180)
    private static double wrapDeg180(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

    /**
     * Choose the best equivalent angle (angle + 360*k) that:
     *  - lies within [MIN_ANGLE_DEG, MAX_ANGLE_DEG]
     *  - is closest to current turret angle (min movement)
     * If none fit, returns the clipped angle.
     */
    private double chooseWrappedWithinLimits(double desiredDeg, double currentDeg) {
        double best = Double.NaN;
        double bestCost = Double.POSITIVE_INFINITY;

        // Try a few wraps around 360. (Enough for your ±200° range)
        for (int k = -1; k <= 1; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < MIN_ANGLE_DEG || cand > MAX_ANGLE_DEG) continue;

            double cost = Math.abs(cand - currentDeg);
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }

        if (!Double.isNaN(best)) return best;

        // No wrapped candidate fits -> hard clip
        return Range.clip(desiredDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    /**
     * Compute turret angle (deg) needed to face a world/field point (targetX,targetY),
     * using Pedro coordinates and robot pose heading.
     *
     * Returns a turret-relative angle in degrees (your turret frame),
     * automatically wrapped to stay inside turret limits (±200°) while minimizing motion.
     */
    public double computeAngleToFaceTargetDeg(double targetX, double targetY, Pose robotPose) {
        if (robotPose == null) return getCurrentAngleDeg();

        double rx = robotPose.getX();
        double ry = robotPose.getY();

        double dx = targetX - rx;
        double dy = targetY - ry;

        // If target is basically on top of robot, don't change anything
        if (Math.hypot(dx, dy) < FACE_TARGET_MIN_DIST_IN) {
            return getCurrentAngleDeg();
        }

        // Field bearing to target (rad), assuming Pedro heading convention aligns with atan2(dy,dx)
        double targetFieldRad = Math.atan2(dy, dx);
        double targetFieldDeg = Math.toDegrees(targetFieldRad);

        // Robot heading in field frame (deg)
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Desired turret-relative angle so turret points at that field bearing
        // (turret 0 = robot forward)
        double desiredTurretDeg = wrapDeg180(targetFieldDeg - robotHeadingDeg - TURRET_MOUNT_OFFSET_DEG);

        // Pick best equivalent inside mechanical limits, minimizing motion
        double currentTurretDeg = getCurrentAngleDeg();
        return chooseWrappedWithinLimits(desiredTurretDeg, currentTurretDeg);
    }

    /**
     * Command turret to face a world/field point (targetX,targetY).
     * Call this every loop while "searching" for the tag.
     */
    public void faceTarget(double targetX, double targetY, Pose robotPose) {
        double tgt = computeAngleToFaceTargetDeg(targetX, targetY, robotPose);
        goToAngle(tgt);
    }

    // =========================
    // Update loop
    // =========================
    public void update() {
        // Save a continuous angle for the next OpMode (Auto -> TeleOp)
        PoseStorage.lastTurretAngleDeg = getCurrentAngleDeg();

        // Only resync when basically stopped (prevents offset jumping mid-move)
        if (Math.abs(turretMotor.getVelocity()) < STOP_VELOCITY_TPS) {
            resyncFromAbsIfNeeded();
        }
    }
}

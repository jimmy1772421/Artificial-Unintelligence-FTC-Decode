package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.util.ServoBlock;

@Configurable
public class TurretSubsystemAbsoluteServos {

    // =========================
    // "Motor"/gear constants (now: encoder-shaft-as-motor)
    // =========================

    // Must match WrappedAnalogAbsEncoder.TICKS_PER_REV (keep them consistent!)
    private static final double TICKS_PER_MOTOR_REV = 4096.0;

    // encoder shaft -> turret: 60:280 (shaft gear 60 driving turret gear 280)
    // so shaft rev per turret rev = 280/60
    private static final double MOTOR_REV_PER_TURRET_REV = 280.0 / 60.0;

    private static final double TICKS_PER_TURRET_REV =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV;

    private static final double TICKS_PER_TURRET_DEG = TICKS_PER_TURRET_REV / 360.0;

    // Mechanical window
    private static final double MIN_ANGLE_DEG = -200.0;
    private static final double MAX_ANGLE_DEG =  200.0;

    public static double MAX_AUTO_POWER = 0.2;

    // Small deadband so we don’t spam new targets when dx/dy are tiny
    private static final double FACE_TARGET_MIN_DIST_IN = 0.5;
    private static final double MANUAL_DEADBAND = 0.02;

    // =========================
    // Hardware (ServoBlock)
    // =========================
    private final ServoBlock turretMotor;

    // =========================
    // Incremental reference
    // currentAngleDeg = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
    // =========================
    private double angleOffsetDeg = 0.0;

    // Control state
    private double targetAngleDeg = 0.0;
    private boolean trackEnabled = false;

    // Turret mount offset if turret 0 is not perfectly robot-forward
    private static final double TURRET_MOUNT_OFFSET_DEG = 0.0;

    public TurretSubsystemAbsoluteServos(HardwareMap hardwareMap) {

        turretMotor = new ServoBlock(
                hardwareMap,
                "turretServoA",
                "turretServoB",
                "turretAbs"
        );

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // DO NOT reset encoder here if you want Auto -> TeleOp continuity.
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Restore offset from previous OpMode if available
        if (OpModeStorage.turretAngleOffsetDeg != null) {
            angleOffsetDeg = OpModeStorage.turretAngleOffsetDeg;
        } else {
            // Default: define "current position is 0°" until you call homeHereAsZero()
            angleOffsetDeg = - (turretMotor.getCurrentPosition() / TICKS_PER_TURRET_DEG);
        }

        if (OpModeStorage.turretTrackEnabled != null) {
            trackEnabled = OpModeStorage.turretTrackEnabled;
        }

        targetAngleDeg = getCurrentAngleDeg();
    }

    // =========================
    // Reference / homing
    // =========================

    /** Call this once after you manually aim turret straight forward. */
    public void homeHereAsZero() {
        setAngleReferenceDeg(0.0);
    }

    /**
     * Set what the current turret angle *should be* without resetting encoders.
     * Example: setAngleReferenceDeg(0) when turret is physically forward.
     */
    public void setAngleReferenceDeg(double currentAngleShouldBeDeg) {
        int ticks = turretMotor.getCurrentPosition();
        double tickAngle = ticks / TICKS_PER_TURRET_DEG;
        angleOffsetDeg = currentAngleShouldBeDeg - tickAngle;

        // persist for next OpMode
        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
    }

    // =========================
    // Tick / angle helpers
    // =========================

    public double getCurrentAngleDeg() {
        int ticks = turretMotor.getCurrentPosition();
        return angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    private int turretDegToTicks(double turretDeg) {
        return (int) Math.round((turretDeg - angleOffsetDeg) * TICKS_PER_TURRET_DEG);
    }

    // Wrap degrees to [-180, 180)
    private static double wrapDeg180(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

    /**
     * Choose best equivalent angle (angle + 360*k) within [MIN, MAX], minimizing motion.
     */
    private double chooseWrappedWithinLimits(double desiredDeg, double currentDeg) {
        double best = Double.NaN;
        double bestCost = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < MIN_ANGLE_DEG || cand > MAX_ANGLE_DEG) continue;

            double cost = Math.abs(cand - currentDeg);
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }

        if (!Double.isNaN(best)) return best;
        return Range.clip(desiredDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    // =========================
    // Manual + position control
    // =========================

    public void setManualPower(double power) {
        power = Range.clip(power, -1.0, 1.0);

        // If driver isn't actually commanding manual motion,
        // DO NOT override RUN_TO_POSITION / holding behavior.
        if (Math.abs(power) < MANUAL_DEADBAND) {
            // If we’re already holding/auto, keep holding.
            if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) return;

            // Otherwise (pure manual mode), just send 0
            turretMotor.setPower(0.0);
            return;
        }

        double current = getCurrentAngleDeg();
        double marginDeg = 5.0;

        if (power > 0 && current >= MAX_ANGLE_DEG - marginDeg) power = 0.0;
        else if (power < 0 && current <= MIN_ANGLE_DEG + marginDeg) power = 0.0;

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(power);
    }

    public void goToAngle(double commandedAngleDeg) {
        double currentDeg = getCurrentAngleDeg();
        targetAngleDeg = chooseWrappedWithinLimits(commandedAngleDeg, currentDeg);

        int targetTicks = turretDegToTicks(targetAngleDeg);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_AUTO_POWER);
    }

    // =========================
    // Tracking helpers
    // =========================
    public void setTrackEnabled(boolean enabled) {
        trackEnabled = enabled;
        OpModeStorage.turretTrackEnabled = enabled;
    }
    public boolean isTrackEnabled() { return trackEnabled; }

    private static final double VISION_DEADBAND_DEG = 0.15;
    private static final double VISION_GAIN = 3.0;
    private static final double VISION_MAX_STEP_DEG = 50.0;

    public void trackWithTxDeg(double txDeg) {
        if (!trackEnabled) return;
        if (Double.isNaN(txDeg)) return;
        if (Math.abs(txDeg) < VISION_DEADBAND_DEG) return;

        double stepDeg = Range.clip(txDeg * VISION_GAIN, -VISION_MAX_STEP_DEG, VISION_MAX_STEP_DEG);
        double newTarget = getCurrentAngleDeg() + stepDeg;
        goToAngle(newTarget);
    }

    /**
     * Compute turret angle (deg) to face a field point (targetX,targetY) given robot pose.
     */
    public double computeAngleToFaceTargetDeg(double targetX, double targetY, Pose robotPose) {
        if (robotPose == null) return getCurrentAngleDeg();

        double rx = robotPose.getX();
        double ry = robotPose.getY();

        double dx = targetX - rx;
        double dy = targetY - ry;

        if (Math.hypot(dx, dy) < FACE_TARGET_MIN_DIST_IN) return getCurrentAngleDeg();

        double targetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // turret 0 = robot forward
        double desiredTurretDeg = -wrapDeg180(targetFieldDeg - robotHeadingDeg - TURRET_MOUNT_OFFSET_DEG);

        return chooseWrappedWithinLimits(desiredTurretDeg, getCurrentAngleDeg());
    }

    public void faceTarget(double targetX, double targetY, Pose robotPose) {
        goToAngle(computeAngleToFaceTargetDeg(targetX, targetY, robotPose));
    }

    public void aimStepDeg(double errorDeg, double deadbandDeg, double maxStepDeg) {
        if (Double.isNaN(errorDeg)) return;

        if (Math.abs(errorDeg) < deadbandDeg) {
            goToAngle(getCurrentAngleDeg()); // hold
            return;
        }

        double step = Range.clip(errorDeg, -maxStepDeg, maxStepDeg);
        goToAngle(getCurrentAngleDeg() + step);
    }

    // =========================
    // Update loop (persist + servo run-to-pos)
    // =========================
    public void update() {
        // Needed so RUN_TO_POSITION actually drives the servos
        turretMotor.update();

        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
        OpModeStorage.turretTrackEnabled = trackEnabled;
    }
}

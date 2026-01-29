package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

@Configurable
public class TurretSubsystemIncremental_Swyft {

    // ===== Hardware names =====
    public static String SERVO_A_NAME = "turretServoA";
    public static String SERVO_B_NAME = "turretServoB";
    public static String ABS_ENCODER_NAME = "turretAbs";

    // ===== ABS config =====
    public static double ABS_ANALOG_RANGE_V = 3.3;
    public static boolean ABS_REVERSED = false;
    public static double ABS_ZERO_OFFSET_DEG = 0.0; // zero offset applied inside encoder wrapper

    // ===== Ratio swap easily =====
    public static double DRIVER_PULLEY_TEETH = 60.0;  // set 60 or 70
    public static double TURRET_PULLEY_TEETH = 280.0;

    // ===== Direction knobs =====
    public static boolean OUTPUT_REVERSED = true; // flips servo power sign (this changes physical direction)
    public static boolean SERVO_A_INVERTED = false;
    public static boolean SERVO_B_INVERTED = false;

    // ===== Limits =====
    public static double MIN_ANGLE_DEG = -200.0;
    public static double MAX_ANGLE_DEG =  200.0;
    public static double ENDSTOP_MARGIN_DEG = 3.0;
    public static double ABS_NOISE_DEADBAND_DEG = 0.15; // start 0.10..0.30 (shaft-deg)


    // ===== PIDF (deg -> power) =====
    public static double kP = 0.006;
    public static double kI = 0.0008;
    public static double kD = 0.00;

    // Feedforward:
    // kS: static friction power (useful for CR servos + belts)
    // kV: power per (deg/s) of desired target velocity (often small or 0)
    public static double kS = 0.07;
    public static double kV = 0.0000;

    public static double I_CLAMP = 0.25;
    public static double HOLD_DEADBAND_DEG = 0.25;
    public static double MAX_AUTO_POWER = 0.7;
    public static double STOP_TOL_DEG = 0.8;      // inside this, stop
    public static double MINPOWER_BAND_DEG = 10.0; // if error is between STOP_TOL and this, force min power
    public static double MIN_POWER = 0.06;        // tune to just start moving reliably


    // ===== Slew rate limiting =====
    // Limit target changes so tracking doesn't step instantly
    public static double MAX_TARGET_RATE_DEG_PER_SEC = 360.0; // try 180..720
    // Limit output changes to prevent oscillation
    public static double MAX_POWER_SLEW_PER_SEC = 2.5;        // try 1..8

    // ===== ABS noise guards =====
    public static double MAX_ABS_STEP_DEG_PER_UPDATE = 40.0; // reject analog spikes

    // ===== Aim =====
    private static final double FACE_TARGET_MIN_DIST_IN = 0.5;
    public static double TURRET_MOUNT_OFFSET_DEG = 0.0;

    public static double VISION_DEADBAND_DEG = 0.15;
    public static double VISION_GAIN = 3.0;
    public static double VISION_MAX_STEP_DEG = 50.0;

    // ===== Hardware =====
    private final AbsoluteAnalogEncoder absEncoder;
    private final CRServoEx servoA;
    private final CRServoEx servoB;

    // ===== ABS unwrap state (shaft degrees) =====
    private double lastAbsDeg0to360 = Double.NaN;
    private double continuousShaftDeg = 0.0;

    // ===== Cached turret angle (deg), updated only in update() =====
    private double currentTurretDeg = 0.0;

    // ===== Reference =====
    private double angleOffsetDeg = 0.0;

    // ===== Target handling =====
    // targetCmdDeg: where you WANT to go (raw)
    // targetDeg: slew-limited internal target used by PIDF
    private double targetCmdDeg = 0.0;
    private double targetDeg = 0.0;
    private double lastTargetDeg = 0.0;

    // ===== Mode =====
    private boolean trackEnabled = false;
    private boolean positionMode = true;
    private double manualPower = 0.0;

    // ===== PID state =====
    private double iTerm = 0.0;
    private double lastErr = 0.0;

    // ===== Output slew state =====
    private double appliedPower = 0.0;

    private long lastTimeNs = 0L;

    // Extra P gain only during vision tracking
    public static double VISION_KP_MULT = 2.0; // start 1.5..4.0
    private boolean visionKpBoostEnabled = false;
    public static double VISION_MIN_POWER = 0.10; // tune: 0.06..0.18
    private boolean visionMinPowerEnabled = false;

    public void setVisionMinPowerEnabled(boolean enabled) {
        visionMinPowerEnabled = enabled;
    }

    public void setVisionKpBoostEnabled(boolean enabled) {
        visionKpBoostEnabled = enabled;
    }


    public TurretSubsystemIncremental_Swyft(HardwareMap hardwareMap) {
        absEncoder = new AbsoluteAnalogEncoder(hardwareMap, ABS_ENCODER_NAME, ABS_ANALOG_RANGE_V, AngleUnit.DEGREES)
                .setReversed(ABS_REVERSED)
                .zero(ABS_ZERO_OFFSET_DEG);

        // We use RawPower because we want our own multi-turn/window control
        servoA = new CRServoEx(hardwareMap, SERVO_A_NAME, absEncoder, CRServoEx.RunMode.RawPower);
        servoB = new CRServoEx(hardwareMap, SERVO_B_NAME, absEncoder, CRServoEx.RunMode.RawPower);

        servoA.setInverted(SERVO_A_INVERTED);
        servoB.setInverted(SERVO_B_INVERTED);

        if (OpModeStorage.turretAngleOffsetDeg != null) angleOffsetDeg = OpModeStorage.turretAngleOffsetDeg;
        if (OpModeStorage.turretTrackEnabled != null) trackEnabled = OpModeStorage.turretTrackEnabled;

        double absNow = readAbs0to360();

        if (OpModeStorage.turretShaftContinuousDeg != null) {
            double stored = OpModeStorage.turretShaftContinuousDeg;
            continuousShaftDeg = absNow + 360.0 * Math.round((stored - absNow) / 360.0);
        } else {
            continuousShaftDeg = absNow;
        }

        lastAbsDeg0to360 = absNow;

        double turretFromEncoder = continuousShaftDeg / shaftRevPerTurretRev();

        if (!OpModeStorage.turretInitDone) {
            // FIRST OpMode in this RC app session: define "current position = 0°"
            angleOffsetDeg = -turretFromEncoder;
            OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
            OpModeStorage.turretInitDone = true;
        } else {
            // After first OpMode: NEVER change the reference automatically
            if (OpModeStorage.turretAngleOffsetDeg != null) {
                angleOffsetDeg = OpModeStorage.turretAngleOffsetDeg;
            }
        }

        recomputeTurretAngleCached();

        targetCmdDeg = currentTurretDeg;
        targetDeg = currentTurretDeg;
        lastTargetDeg = targetDeg;

        lastTimeNs = System.nanoTime();
        setServos(0.0);
    }

    // =========================
    // Public getters (NO unwrap here)
    // =========================
    public double getCurrentAngleDeg() { return currentTurretDeg; }
    public double getTargetAngleDeg() { return targetDeg; }

    public void setTrackEnabled(boolean enabled) {
        trackEnabled = enabled;
        OpModeStorage.turretTrackEnabled = enabled;
    }
    public boolean isTrackEnabled() { return trackEnabled; }

    public void homeHereAsZero() {
//        setAngleReferenceDeg(0.0);
    }

    public void setAngleReferenceDeg(double currentAngleShouldBeDeg) {
//        angleOffsetDeg += (currentAngleShouldBeDeg - currentTurretDeg);
//        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
//
//        targetCmdDeg = currentAngleShouldBeDeg;
//        targetDeg = currentAngleShouldBeDeg;
//        resetPid();
    }

    public void setManualPower(double power) {
        positionMode = false;
        manualPower = Range.clip(power, -1.0, 1.0);

        if (manualPower > 0 && currentTurretDeg >= MAX_ANGLE_DEG - ENDSTOP_MARGIN_DEG) manualPower = 0.0;
        if (manualPower < 0 && currentTurretDeg <= MIN_ANGLE_DEG + ENDSTOP_MARGIN_DEG) manualPower = 0.0;
    }

    public void goToAngle(double commandedAngleDeg) {
        targetCmdDeg = chooseWrappedWithinLimits(commandedAngleDeg, currentTurretDeg);
        positionMode = true;
        resetPid();
    }

    public void trackWithTxDeg(double txDeg) {
        if (!trackEnabled) return;
        if (Double.isNaN(txDeg)) return;
        if (Math.abs(txDeg) < VISION_DEADBAND_DEG) return;

        double stepDeg = Range.clip(txDeg * VISION_GAIN, -VISION_MAX_STEP_DEG, VISION_MAX_STEP_DEG);
        targetCmdDeg = chooseWrappedWithinLimits(currentTurretDeg + stepDeg, currentTurretDeg);
        positionMode = true;
        // NOTE: no PID reset here; tracking should be smooth
    }

    public double computeAngleToFaceTargetDeg(double targetX, double targetY, Pose robotPose) {
        if (robotPose == null) return currentTurretDeg;

        double rx = robotPose.getX();
        double ry = robotPose.getY();

        double dx = targetX - rx;
        double dy = targetY - ry;

        if (Math.hypot(dx, dy) < FACE_TARGET_MIN_DIST_IN) return currentTurretDeg;

        double targetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        double desiredTurretDeg = -wrapDeg180(targetFieldDeg - robotHeadingDeg - TURRET_MOUNT_OFFSET_DEG);
        return chooseWrappedWithinLimits(desiredTurretDeg, currentTurretDeg);
    }

    public void faceTarget(double targetX, double targetY, Pose robotPose) {
        goToAngle(computeAngleToFaceTargetDeg(targetX, targetY, robotPose));
    }

    // =========================
    // Update (call ONCE per loop)
    // =========================
    public void update() {
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0L) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        if (dt <= 1e-4) dt = 0.02;

        // 1) unwrap ABS once per loop
        unwrapStep(dt);

        // 2) cache turret angle once per loop
        recomputeTurretAngleCached();

        // 3) slew-limit target
        lastTargetDeg = targetDeg;
        targetDeg = slewTowards(targetDeg, targetCmdDeg, MAX_TARGET_RATE_DEG_PER_SEC, dt);
        double desiredTargetVelDegPerSec = (targetDeg - lastTargetDeg) / dt;

        // 4) compute desired power
        double desiredPower = positionMode
                ? computePidfPower(targetDeg, desiredTargetVelDegPerSec, dt)
                : manualPower;

        // 5) endstop clamp
        if (desiredPower > 0 && currentTurretDeg >= MAX_ANGLE_DEG) desiredPower = 0.0;
        if (desiredPower < 0 && currentTurretDeg <= MIN_ANGLE_DEG) desiredPower = 0.0;

        // 6) output slew
        appliedPower = slewTowards(appliedPower, desiredPower, MAX_POWER_SLEW_PER_SEC, dt);

        // 7) apply
        setServos(appliedPower);

        // persist
        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
        OpModeStorage.turretTrackEnabled = trackEnabled;
        OpModeStorage.turretShaftContinuousDeg = continuousShaftDeg;
    }

    // =========================
    // PIDF + helpers
    // =========================
    private void resetPid() {
        iTerm = 0.0;
        lastErr = 0.0;
    }

    private double computePidfPower(double targetDeg, double desiredVelDegPerSec, double dt) {
        double err = targetDeg - currentTurretDeg;

        // Stop buzzing near target
        if (Math.abs(err) < HOLD_DEADBAND_DEG) {
            resetPid();
            return 0.0;
        }

        iTerm += err * dt;
        iTerm = Range.clip(iTerm, -I_CLAMP, I_CLAMP);

        double dErr = (err - lastErr) / dt;
        lastErr = err;

        double kpEff = kP * (visionKpBoostEnabled ? VISION_KP_MULT : 1.0);
        double out = (kpEff * err) + (kI * iTerm) + (kD * dErr);

        // kS only when we actually want motion
        out += Math.signum(err) * kS;

        // velocity feedforward from slew-limited target velocity
        out += kV * desiredVelDegPerSec;

        // after computing out (and adding kS/kV), before clipping:
        double aerr = Math.abs(err);

    // If we’re outside stop tolerance but still “small-ish”, force enough power to move
        if (aerr > STOP_TOL_DEG && aerr < MINPOWER_BAND_DEG) {
            out = Math.signum(err) * Math.max(Math.abs(out), MIN_POWER);
        }
        if (visionMinPowerEnabled && aerr > STOP_TOL_DEG) {
            out = Math.signum(err) * Math.max(Math.abs(out), VISION_MIN_POWER);
        }

        return Range.clip(out, -MAX_AUTO_POWER, MAX_AUTO_POWER);
    }

    private static double slewTowards(double current, double target, double ratePerSec, double dt) {
        double maxStep = Math.abs(ratePerSec) * dt;
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    private void setServos(double power) {
        power = Range.clip(power, -1.0, 1.0);
        if (OUTPUT_REVERSED) power = -power;
        servoA.set(power);
        servoB.set(power);
    }

    // ===== ABS unwrap / angle math =====
    private double readAbs0to360() {
        double deg = absEncoder.getCurrentPosition();
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private void unwrapStep(double dt) {
        double absDeg = readAbs0to360();

        if (Double.isNaN(lastAbsDeg0to360)) {
            lastAbsDeg0to360 = absDeg;
            continuousShaftDeg = absDeg;
            return;
        }

        // Snap absDeg to the nearest 360*k branch around our last continuous value
        double snapped = absDeg + 360.0 * Math.round((continuousShaftDeg - absDeg) / 360.0);
        double delta = snapped - continuousShaftDeg;

        // Kill tiny jitter so we don't "drift" at rest
        if (Math.abs(delta) < ABS_NOISE_DEADBAND_DEG) {
            lastAbsDeg0to360 = absDeg; // keep last updated
            return;                   // don't change continuousShaftDeg
        }

        // Spike / impossible jump guard
        // With ~35ms loop, 120 deg/update corresponds to ~3428 deg/s shaft, usually plenty.
        if (Math.abs(delta) > MAX_ABS_STEP_DEG_PER_UPDATE) {
            // resync the raw absolute tracker, but don't integrate the jump
            lastAbsDeg0to360 = absDeg;
            return;
        }

        continuousShaftDeg = snapped;
        lastAbsDeg0to360 = absDeg;
    }


    private double shaftRevPerTurretRev() {
        return TURRET_PULLEY_TEETH / DRIVER_PULLEY_TEETH; // 280/70=4, 280/60=4.6667
    }

    private void recomputeTurretAngleCached() {
        double turretFromEncoder = continuousShaftDeg / shaftRevPerTurretRev();
        currentTurretDeg = angleOffsetDeg + turretFromEncoder;
    }

    private static double wrapDeg180(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

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
    public void setTargetAngleDegNoReset(double commandedAngleDeg) {
        targetCmdDeg = chooseWrappedWithinLimits(commandedAngleDeg, currentTurretDeg);
        positionMode = true;
    }
    public void holdCurrentNoReset() {
        positionMode = true;
        targetCmdDeg = chooseWrappedWithinLimits(currentTurretDeg, currentTurretDeg);
        // do NOT reset PID
    }

    public void holdTargetNoReset() {
        positionMode = true;
        targetCmdDeg = chooseWrappedWithinLimits(targetDeg, currentTurretDeg);
        // do NOT reset PID
    }

}

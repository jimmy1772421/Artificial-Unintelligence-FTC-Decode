package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.OpModeStorage;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class TurretSubsystemAbsoluteServos {

    // =========================
    // Servo + belt constants
    // =========================
    public static int DRIVER_PULLEY_TEETH = 60;      // 60 or 70
    public static int TURRET_PULLEY_TEETH = 280;     // turret pulley

    // =========================
    // Mechanical window
    // =========================
    public static double MIN_ANGLE_DEG = -200.0;
    public static double MAX_ANGLE_DEG =  200.0;

    public static double MAX_AUTO_POWER = 0.85;

    private static final double FACE_TARGET_MIN_DIST_IN = 0.5;
    private static final double TURRET_MOUNT_OFFSET_DEG = 0.0;

    // =========================
    // Servo config / direction
    // =========================
    public static boolean SERVO_A_INVERTED = false;
    public static boolean SERVO_B_INVERTED = false;
    public static boolean OUTPUT_REVERSED = true;

    // =========================
    // Absolute analog encoder config
    // =========================
    public static double ABS_ANALOG_RANGE_V = 3.3;
    public static boolean ABS_REVERSED = false;
    public static double ABS_ZERO_OFFSET_DEG = 0.0;

    // =========================
    // PIDF (power output in [-1, 1])
    // =========================
    public static double KP = 0.020;
    public static double KI = 0.000;
    public static double KD = 0.001;
    public static double KF = 0.000;

    // Servo deadzone / stiction helpers
    public static double MIN_EFFECTIVE_POWER = 0.06;  // signed minimum we apply ourselves
    public static double KS = 0.0;                    // optional extra static term (signed)

    public static double POSITION_TOL_DEG = 0.25;

    // =========================
    // Vision tracking (same behavior as old)
    // =========================
    public static double VISION_DEADBAND_DEG = 0.15;
    public static double VISION_GAIN = 3.0;
    public static double VISION_MAX_STEP_DEG = 50.0;

    // =========================
    // Hardware
    // =========================
    private final CRServoEx turretServoA;
    private final CRServoEx turretServoB;
    private final AbsoluteAnalogEncoder turretAbs;

    // =========================
    // Absolute -> continuous “incremental-like” angle
    // =========================
    private boolean absInitialized = false;
    private double lastAbsDeg0to360 = 0.0;
    private double continuousAbsDeg = 0.0; // multi-turn

    // currentAngleDeg = angleOffsetDeg + continuousAbsDeg
    private double angleOffsetDeg = 0.0;

    // Control state
    private double targetAngleDeg = 0.0;
    private boolean trackEnabled = false;

    private final PIDFController pid = new PIDFController(KP, KI, KD, KF);

    // ===== Option A: manual override state =====
    private boolean manualOverride = false;
    private double manualPowerCmd = 0.0;
    public static double MANUAL_DEADBAND = 0.05;   // stick noise cutoff
    public static double MANUAL_HYST = 0.02;       // optional anti-chatter
    public static double MIN_POWER_ERROR_DEG = 8.0;
    public static double SETTLE_ZONE_DEG = 3.0;       // Let PID be gentle when close

    public TurretSubsystemAbsoluteServos(HardwareMap hardwareMap) {
        turretServoA = new CRServoEx(hardwareMap, "turretServoA").setRunMode(CRServoEx.RunMode.RawPower);
        turretServoB = new CRServoEx(hardwareMap, "turretServoB").setRunMode(CRServoEx.RunMode.RawPower);

        turretServoA.setInverted(SERVO_A_INVERTED);
        turretServoB.setInverted(SERVO_B_INVERTED);

        turretAbs = new AbsoluteAnalogEncoder(hardwareMap, "turretAbs", ABS_ANALOG_RANGE_V, AngleUnit.DEGREES);
        turretAbs.setReversed(ABS_REVERSED);
        turretAbs.zero(ABS_ZERO_OFFSET_DEG);

        updateAbsUnwrap();

        if (OpModeStorage.turretAngleOffsetDeg != null) {
            angleOffsetDeg = OpModeStorage.turretAngleOffsetDeg;
        } else {
            angleOffsetDeg = -continuousAbsDeg;
        }

        if (OpModeStorage.turretTrackEnabled != null) {
            trackEnabled = OpModeStorage.turretTrackEnabled;
        }

        targetAngleDeg = getCurrentAngleDeg();

        pid.setTolerance(POSITION_TOL_DEG);
        pid.reset();
    }

    // =========================
    // Reference / homing
    // =========================
    public void homeHereAsZero() { setAngleReferenceDeg(0.0); }

    public void setAngleReferenceDeg(double currentAngleShouldBeDeg) {
        updateAbsUnwrap();
        angleOffsetDeg = currentAngleShouldBeDeg - continuousAbsDeg;

        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;

        // hold here
        targetAngleDeg = getCurrentAngleDeg();
        manualOverride = false;
        pid.reset();
    }

    // =========================
    // Absolute unwrap logic
    // =========================
    private void updateAbsUnwrap() {
        double absDeg = turretAbs.getCurrentPosition();
        absDeg = absDeg % 360.0;
        if (absDeg < 0) absDeg += 360.0;

        if (!absInitialized) {
            absInitialized = true;
            lastAbsDeg0to360 = absDeg;
            continuousAbsDeg = chooseWrappedWithinLimits(absDeg, 0.0);
            return;
        }

        double delta = absDeg - lastAbsDeg0to360;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        continuousAbsDeg += delta;
        lastAbsDeg0to360 = absDeg;
    }

    // =========================
    // Angle helpers
    // =========================
    public double getCurrentAngleDeg() {
        updateAbsUnwrap();
        return angleOffsetDeg + continuousAbsDeg;
    }

    public double getTargetAngleDeg() { return targetAngleDeg; }

    public double getContinuousAbsDeg() {
        updateAbsUnwrap();
        return continuousAbsDeg;
    }

    public double getAbsDeg0to360() { return lastAbsDeg0to360; }

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

    // =========================
    // Servo output helper
    // =========================
    private void setTurretPower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        if (OUTPUT_REVERSED) power = -power;
        turretServoA.set(power);
        turretServoB.set(power);
    }

    // =========================
    // Manual + position control
    // =========================
    public void setManualPower(double power) {
        double current = getCurrentAngleDeg();
        double marginDeg = 5.0;

        // Hysteresis deadband (prevents rapid toggling)
        double enter = MANUAL_DEADBAND;
        double exit  = Math.max(0.0, MANUAL_DEADBAND - MANUAL_HYST);

        boolean wantManual = manualOverride
                ? (Math.abs(power) > exit)
                : (Math.abs(power) > enter);

        if (!wantManual) {
            // If we were manual, latch hold target ONCE
            if (manualOverride) {
                manualOverride = false;
                targetAngleDeg = getCurrentAngleDeg(); // hold exactly here
                pid.reset();                           // reset once, not every loop
            }
            manualPowerCmd = 0.0;
            ;
        }

        // Still manual
        // (Optional) rescale so it doesn't jump at deadband edge
        double sign = Math.signum(power);
        double mag  = (Math.abs(power) - enter) / (1.0 - enter);
        power = sign * Range.clip(mag, 0.0, 1.0);

        // Edge safety
        if (power > 0 && current >= MAX_ANGLE_DEG - marginDeg) power = 0.0;
        else if (power < 0 && current <= MIN_ANGLE_DEG + marginDeg) power = 0.0;

        manualOverride = true;
        manualPowerCmd = Range.clip(power, -1.0, 1.0);
    }

    public void goToAngle(double commandedAngleDeg) {
        double currentDeg = getCurrentAngleDeg();
        targetAngleDeg = chooseWrappedWithinLimits(commandedAngleDeg, currentDeg);
        targetAngleDeg = Range.clip(targetAngleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        manualOverride = false;
        pid.reset();
        pid.setTolerance(POSITION_TOL_DEG);
    }

    // =========================
    // Tracking helpers
    // =========================
    public void setTrackEnabled(boolean enabled) {
        trackEnabled = enabled;
        OpModeStorage.turretTrackEnabled = enabled;
    }
    public boolean isTrackEnabled() { return trackEnabled; }

    public void trackWithTxDeg(double txDeg) {
        if (!trackEnabled) return;
        if (Double.isNaN(txDeg)) return;
        if (Math.abs(txDeg) < VISION_DEADBAND_DEG) return;

        double stepDeg = Range.clip(txDeg * VISION_GAIN, -VISION_MAX_STEP_DEG, VISION_MAX_STEP_DEG);
        goToAngle(getCurrentAngleDeg() + stepDeg);
    }

    public double computeAngleToFaceTargetDeg(double targetX, double targetY, Pose robotPose) {
        if (robotPose == null) return getCurrentAngleDeg();

        double rx = robotPose.getX();
        double ry = robotPose.getY();

        double dx = targetX - rx;
        double dy = targetY - ry;

        if (Math.hypot(dx, dy) < FACE_TARGET_MIN_DIST_IN) return getCurrentAngleDeg();

        double targetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        double desiredTurretDeg = -wrapDeg180(targetFieldDeg - robotHeadingDeg - TURRET_MOUNT_OFFSET_DEG);
        return chooseWrappedWithinLimits(desiredTurretDeg, getCurrentAngleDeg());
    }

    public void faceTarget(double targetX, double targetY, Pose robotPose) {
        goToAngle(computeAngleToFaceTargetDeg(targetX, targetY, robotPose));
    }

    // =========================
    // Update loop (manual OR PID)
    // =========================
    public void update() {
        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
        OpModeStorage.turretTrackEnabled = trackEnabled;

        // If driver is commanding manual, do ONLY manual power
        if (manualOverride) {
            // still enforce hard window
            double current = getCurrentAngleDeg();
            if ((current >= MAX_ANGLE_DEG && manualPowerCmd > 0) ||
                    (current <= MIN_ANGLE_DEG && manualPowerCmd < 0)) {
                setTurretPower(0.0);
            } else {
                setTurretPower(manualPowerCmd);
            }
            return;
        }

        // PID hold/position
        pid.setPIDF(KP, KI, KD, KF);
        pid.setTolerance(POSITION_TOL_DEG);

        double current = getCurrentAngleDeg();
        double error = targetAngleDeg - current;

        // stop if basically at target
        if (Math.abs(error) <= POSITION_TOL_DEG) {
            setTurretPower(0.0);
            return;
        }

        // edge safety
        if ((current >= MAX_ANGLE_DEG && error > 0) || (current <= MIN_ANGLE_DEG && error < 0)) {
            setTurretPower(0.0);
            return;
        }

        // PIDF output (keep your original call)
        double out = pid.calculate(current, targetAngleDeg);

// Only boost power when error is large
        if (Math.abs(error) > 10.0 && Math.abs(out) < MIN_EFFECTIVE_POWER) {
            out = Math.copySign(MIN_EFFECTIVE_POWER, error);
        }

// When close, trust the PID completely
        out = Range.clip(out, -MAX_AUTO_POWER, MAX_AUTO_POWER);
        setTurretPower(out);

        double pidOut = pid.calculate(current, targetAngleDeg);

// Log these every loop:
        telemetry.addData("DEBUG error", error);
        telemetry.addData("DEBUG pidOut", pidOut);
        telemetry.addData("DEBUG current", current);
        telemetry.addData("DEBUG target", targetAngleDeg);
    }

    public void stop() {
        manualOverride = false;
        setTurretPower(0.0);
        goToAngle(getCurrentAngleDeg());
    }
}

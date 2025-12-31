package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpindexerSubsystem_State {

    public enum Ball { EMPTY, GREEN, PURPLE, UNKNOWN }

    private enum RawColor { RED, GREEN, PURPLE }

    // ===== Geometry =====
    private static final double TICKS_PER_REV = 383.6;     // goBILDA 435rpm integrated
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;
    private static final double LOAD_ANGLE = 180.0;
    private static final int SLOT_COUNT = 3;

    // ===== Abs encoder =====
    private static final double ABS_VREF = 3.3;
    private static final double ABS_MECH_OFFSET_DEG = 245.0;   // calibrate
    private static final double ABS_REZERO_THRESHOLD_DEG = 5.0;

    // ===== Internal state =====
    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    private final Ball[] slots = new Ball[SLOT_COUNT];

    private int zeroTicks = 0;
    private int intakeIndex = 0;

    // ===== TARGET ANGLE - The ONE thing we chase =====
    private double targetAngleDeg = 0.0;
    private int cachedTargetTicks = 0;

    // Auto intake flags
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;

    // Eject flag (used by teleop)
    private boolean ejecting = false;

    // Manual intake queue (not fully implemented, but kept for future use)
    private boolean manualIntakeQueued = false;
    private int manualIntakeQueuedTag = 0;

    // Pattern tag / sequencer
    private int gameTag = 0;
    private int patternStep = 0;

    // ===== PIDF caching =====
    private PIDFCoefficients lastPos = new PIDFCoefficients(0, 0, 0, 0);
    private PIDFCoefficients lastVel = new PIDFCoefficients(0, 0, 0, 0);

    // ===== Eject state machine (simple, non-blocking stub) =====
    private enum EjectState {
        IDLE,
        EJECTING
    }

    private EjectState ejectState = EjectState.IDLE;
    private long ejectEndTimeMs = 0;

    // ===== ABS correction throttling =====
    private long lastAbsCheckMs = 0;
    private static final long ABS_CHECK_INTERVAL_MS = 500;  // Check every 500ms

    public SpindexerSubsystem_State(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");
        absEncoder = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
        }

        autoZeroFromAbs();

        // Initial target: slot 0 at intake
        targetAngleDeg = slotCenterAngleAtIntake(0);
        updateTargetTicks();
    }

    // =========================
    // PUBLIC API
    // =========================
    public void setGameTag(int tag) { this.gameTag = tag; }
    public int getGameTag() { return gameTag; }

    public Ball[] getSlots() { return slots; }
    public int getEncoder() { return motor.getCurrentPosition(); }
    public int getTarget() { return cachedTargetTicks; }
    public int getIntakeSlotIndex() { return intakeIndex; }

    public boolean isMoving() {
        int err = Math.abs(cachedTargetTicks - motor.getCurrentPosition());
        return err > SpindexerTuningConfig_State.HOLD_DEADBAND_TICKS;
    }

    public boolean isAutoRotating() { return pendingAutoRotate; }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public void clearSlot(int slotIndex) { slots[slotIndex] = Ball.EMPTY; }
    public boolean slotHasBall(int slotIndex) { return slots[slotIndex] != Ball.EMPTY; }

    // ===== Commands =====
    public void cmdIntakeSlot(int slot) {
        slot = ((slot % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        setTargetAngle(slotCenterAngleAtIntake(slot), slot);
    }

    public void cmdLoadSlot(int slot) {
        slot = ((slot % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        double angle = slotCenterAngleAtIntake(slot) + (LOAD_ANGLE - INTAKE_ANGLE);
        setTargetAngle(angle, -1);
    }

    public void cmdAngle(double angleDeg) {
        setTargetAngle(angleDeg, -1);
    }

    // =========================
    // TELEOP COMPAT WRAPPERS
    // =========================

    /** Non-blocking home: put slot 0 at intake. */
    public void homeToIntake() {
        cmdIntakeSlot(0);
    }

    /** True while the eject state machine is active. */
    public boolean isEjecting() {
        return ejecting;
    }

    /**
     * What the current AprilTag pattern means (for telemetry).
     */
    public String getGamePattern() {
        Ball[] pattern = getPatternForTag(gameTag);
        if (pattern == null) return "Fast (no pattern, tag=" + gameTag + ")";

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length; i++) {
            if (i > 0) sb.append("-");
            if (pattern[i] == Ball.PURPLE) sb.append("P");
            else if (pattern[i] == Ball.GREEN) sb.append("G");
            else sb.append("?");
        }
        return sb.toString();
    }

    /**
     * Main state-machine update (call every loop from TeleOp).
     */
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        // Allow driver override of gameTag
        if (patternTagOverride != 0) {
            this.gameTag = patternTagOverride;
        }

        // Core motion logic
        periodic();

        // Simple non-blocking eject stub
        long now = System.currentTimeMillis();

        if (yEdge && ejectState == EjectState.IDLE && hasAnyBall()) {
            ejectState = EjectState.EJECTING;
            ejecting = true;
            ejectEndTimeMs = now + 500;
        }

        if (ejectState == EjectState.EJECTING && now >= ejectEndTimeMs) {
            ejectState = EjectState.IDLE;
            ejecting = false;
        }

        return isFull();
    }

    // =========================
    // PERIODIC (call every loop) - THE CORE
    // =========================
    public void periodic() {
        applyPidfIfChanged();
        updateAutoRotate();
        periodicAbsCheck();
        chaseTarget();
    }

    // =========================
    // TARGET ANGLE MANAGEMENT
    // =========================
    private void setTargetAngle(double angleDeg, int newIntakeIndex) {
        targetAngleDeg = angleDeg;
        if (newIntakeIndex >= 0) {
            intakeIndex = newIntakeIndex;
        }
        updateTargetTicks();
    }

    private void updateTargetTicks() {
        cachedTargetTicks = shortestTicksToAngle(targetAngleDeg);
    }

    // =========================
    // CHASE TARGET (runs every loop)
    // =========================
    private void chaseTarget() {
        int current = motor.getCurrentPosition();
        int err = cachedTargetTicks - current;

        if (Math.abs(err) <= SpindexerTuningConfig_State.HOLD_DEADBAND_TICKS) {
            // Within deadband - stop buzzing
            if (SpindexerTuningConfig_State.HOLD_ENABLED) {
                motor.setPower(0.0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return;
        }

        // Chase the target
        motor.setTargetPosition(cachedTargetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Use different power based on whether we're moving or holding
        double power = Math.abs(err) > 50
                ? SpindexerTuningConfig_State.MOVE_POWER
                : SpindexerTuningConfig_State.HOLD_POWER;

        motor.setPower(power);
    }

    // =========================
    // Auto rotate (for intake behavior)
    // =========================
    private void updateAutoRotate() {
        if (!pendingAutoRotate) return;

        long now = System.currentTimeMillis();
        if (now < autoRotateTimeMs) return;

        // Don't start auto-rotate if we're still moving from a previous command
        if (isMoving()) return;

        pendingAutoRotate = false;
        int nextIndex = (intakeIndex + 1) % SLOT_COUNT;
        setTargetAngle(slotCenterAngleAtIntake(nextIndex), nextIndex);
    }

    // =========================
    // Manual intake helper
    // =========================
    public void intakeOne(Telemetry telemetry, int tag) {
        this.gameTag = tag;
        manualIntakeQueued = true;
        manualIntakeQueuedTag = tag;
        if (telemetry != null) {
            telemetry.addData("ManualIntake", "Queued intakeOne(tag=%d)", tag);
        }
    }

    // =========================
    // FORCE intake slot color
    // =========================
    public void forceIntakeSlotGreen(Telemetry telemetry) {
        forceIntakeSlotColor(telemetry, Ball.GREEN);
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        forceIntakeSlotColor(telemetry, Ball.PURPLE);
    }

    private void forceIntakeSlotColor(Telemetry telemetry, Ball forcedColor) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = forcedColor;
            if (telemetry != null) {
                telemetry.addData("ForceIntake", "Slot %d forced to %s", intakeIndex, forcedColor);
            }
            pendingAutoRotate = true;
            autoRotateTimeMs = System.currentTimeMillis() + 100;
        } else {
            if (telemetry != null) {
                telemetry.addData("ForceIntake",
                        "Slot %d not empty (=%s)", intakeIndex, slots[intakeIndex]);
            }
        }
    }

    // =========================
    // Angle helpers
    // =========================
    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private double getAbsAngleDeg() {
        double v = absEncoder.getVoltage();
        double angle = (v / ABS_VREF) * 360.0;
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double normalizeAngle(double angleDeg) {
        return (angleDeg % 360.0 + 360.0) % 360.0;
    }

    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    private void autoZeroFromAbs() {
        double absRaw = getAbsAngleDeg();
        double internalAngle = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);

        int currentTicks = motor.getCurrentPosition();
        int internalTicks = (int) Math.round((internalAngle / 360.0) * TICKS_PER_REV);
        zeroTicks = currentTicks - internalTicks;

        double slotIndexF = internalAngle / DEGREES_PER_SLOT;
        int nearestIndex = (int) Math.round(slotIndexF) % SLOT_COUNT;
        if (nearestIndex < 0) nearestIndex += SLOT_COUNT;
        intakeIndex = nearestIndex;
    }

    private double ticksToAngle(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        double angle = revs * 360.0;
        return normalizeAngle(angle);
    }

    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
    }

    private int shortestTicksToAngle(double angleDeg) {
        double norm = normalizeAngle(angleDeg);
        double desiredRevs = norm / 360.0;
        int baseTicks = zeroTicks + (int) Math.round(desiredRevs * TICKS_PER_REV);

        int current = motor.getCurrentPosition();
        int revTicks = (int) Math.round(TICKS_PER_REV);

        // Find the equivalent position closest to current
        // We want: baseTicks + k * revTicks where k minimizes distance
        int diff = current - baseTicks;
        int k = Math.round((float) diff / revTicks);

        return baseTicks + k * revTicks;
    }

    private void periodicAbsCheck() {
        long now = System.currentTimeMillis();
        if (now - lastAbsCheckMs < ABS_CHECK_INTERVAL_MS) return;

        lastAbsCheckMs = now;
        verifyAndCorrectFromAbs();
    }

    private void verifyAndCorrectFromAbs() {
        double absRaw = getAbsAngleDeg();
        double absInternal = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);
        double encInternal = getCurrentAngleDeg();
        double diff = smallestAngleDiff(encInternal, absInternal);

        if (Math.abs(diff) > ABS_REZERO_THRESHOLD_DEG) {
            autoZeroFromAbs();
            // After zero change, recalculate target ticks for same angle
            updateTargetTicks();
        }
    }

    // =========================
    // Color helpers
    // =========================
    private boolean isBallPresent() {
        final double THRESH_CM = 5.0;

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        return present1 || present2;
    }

    private RawColor classifyColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int sum = r + g + b;
        if (sum < 50) return RawColor.RED;

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        boolean greenDominant = (gn > rn + 0.05) && (gn > bn + 0.02);
        boolean redDominant   = (rn > gn + 0.10) && (rn > bn + 0.05);

        if (greenDominant) return RawColor.GREEN;
        if (redDominant) return RawColor.RED;
        return RawColor.PURPLE;
    }

    private Ball detectBallColor() {
        final double THRESH_CM = 5.0;

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        if (!present1 && !present2) return Ball.EMPTY;

        RevColorSensorV3 chosen;
        if (present1 && present2) {
            chosen = (d1 <= d2) ? intakeColor : intakeColor2;
        } else if (present1) {
            chosen = intakeColor;
        } else {
            chosen = intakeColor2;
        }

        RawColor raw = classifyColor(chosen);
        if (raw == RawColor.GREEN) return Ball.GREEN;
        if (raw == RawColor.PURPLE) return Ball.PURPLE;
        return Ball.EMPTY;
    }

    // =========================
    // Debug
    // =========================
    public void debugAbsAngle(Telemetry telemetry) {
        double raw = getAbsAngleDeg();
        double corrected = normalizeAngle(raw - ABS_MECH_OFFSET_DEG);
        double encAngle = getCurrentAngleDeg();
        telemetry.addData("Abs raw", "%.1f deg", raw);
        telemetry.addData("Abs corr", "%.1f deg", corrected);
        telemetry.addData("Enc angle", "%.1f deg", encAngle);
        telemetry.addData("Abs offset", "%.1f deg", ABS_MECH_OFFSET_DEG);
        telemetry.addData("Intake slot index", intakeIndex);
        telemetry.addData("Target angle", "%.1f deg", targetAngleDeg);
        telemetry.addData("Target ticks", cachedTargetTicks);
    }

    // =========================
    // Apply PIDF from config
    // =========================
    private void applyPidfIfChanged() {
        PIDFCoefficients pos = new PIDFCoefficients(
                SpindexerTuningConfig_State.POS_P,
                SpindexerTuningConfig_State.POS_I,
                SpindexerTuningConfig_State.POS_D,
                SpindexerTuningConfig_State.POS_F
        );

        if (pos.p != lastPos.p || pos.i != lastPos.i || pos.d != lastPos.d || pos.f != lastPos.f) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pos);
            lastPos = pos;
        }

        PIDFCoefficients vel = new PIDFCoefficients(
                SpindexerTuningConfig_State.VEL_P,
                SpindexerTuningConfig_State.VEL_I,
                SpindexerTuningConfig_State.VEL_D,
                SpindexerTuningConfig_State.VEL_F
        );

        if (vel.p != lastVel.p || vel.i != lastVel.i || vel.d != lastVel.d || vel.f != lastVel.f) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, vel);
            lastVel = vel;
        }
    }

    // =========================
    // Pattern helper
    // =========================
    private Ball[] getPatternForTag(int tag) {
        switch (tag) {
            case 21:
                return new Ball[]{ Ball.GREEN, Ball.PURPLE, Ball.GREEN };
            case 22:
                return new Ball[]{ Ball.PURPLE, Ball.GREEN, Ball.PURPLE };
            case 23:
                return new Ball[]{ Ball.GREEN, Ball.GREEN, Ball.PURPLE };
            default:
                return null;
        }
    }
}
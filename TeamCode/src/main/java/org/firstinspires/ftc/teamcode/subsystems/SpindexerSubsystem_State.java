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

    // Auto intake flags
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;
    private boolean autoRotateInProgress = false;

    // Eject flag (used by teleop)
    private boolean ejecting = false;

    // Manual intake queue (not fully implemented, but kept for future use)
    private boolean manualIntakeQueued = false;
    private int manualIntakeQueuedTag = 0;

    // Pattern tag / sequencer
    private int gameTag = 0;
    private int patternStep = 0;

    // ===== Motion state machine =====
    private enum MotionState { IDLE, MOVING }
    private MotionState motionState = MotionState.IDLE;
    private long motionDeadlineMs = 0;
    private int pendingIntakeIndexOnFinish = -1;

    // ===== Active hold target =====
    private double holdAngleDeg = 0.0;

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

        // initial hold at slot0 intake
        holdAngleDeg = slotCenterAngleAtIntake(0);
        cmdIntakeSlot(0);
    }

    // =========================
    // PUBLIC API
    // =========================
    public void setGameTag(int tag) { this.gameTag = tag; }
    public int getGameTag() { return gameTag; }

    public Ball[] getSlots() { return slots; }
    public int getEncoder() { return motor.getCurrentPosition(); }
    public int getTarget() { return motor.getTargetPosition(); }
    public int getIntakeSlotIndex() { return intakeIndex; }

    public boolean isMoving() { return motionState != MotionState.IDLE; }
    public boolean isAutoRotating() { return pendingAutoRotate || autoRotateInProgress; }

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

    // ===== Commands for tuner / teleop =====
    public boolean cmdIntakeSlot(int slot) {
        slot = ((slot % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        double angle = slotCenterAngleAtIntake(slot);
        holdAngleDeg = angle;
        return requestMoveToAngle(
                angle,
                SpindexerTuningConfig_State.MOVE_POWER,
                SpindexerTuningConfig_State.MOVE_TIMEOUT_MS,
                slot
        );
    }

    public boolean cmdLoadSlot(int slot) {
        slot = ((slot % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        double angle = slotCenterAngleAtIntake(slot) + (LOAD_ANGLE - INTAKE_ANGLE);
        holdAngleDeg = angle;
        return requestMoveToAngle(
                angle,
                SpindexerTuningConfig_State.MOVE_POWER,
                SpindexerTuningConfig_State.MOVE_TIMEOUT_MS,
                -1
        );
    }

    public boolean cmdAngle(double angleDeg) {
        holdAngleDeg = angleDeg;
        return requestMoveToAngle(
                angleDeg,
                SpindexerTuningConfig_State.MOVE_POWER,
                SpindexerTuningConfig_State.MOVE_TIMEOUT_MS,
                -1
        );
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
     * "G" = green, "P" = purple, "?" = unknown/empty.
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
     *
     * @return true if magazine is full (3 balls).
     */
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        // Allow driver override of gameTag
        if (patternTagOverride != 0) {
            this.gameTag = patternTagOverride;
        }

        // Core motion & hold logic
        periodic();

        // Simple non-blocking eject stub based on Y edge.
        // This keeps "ejecting" valid for teleop shooter logic without blocking.
        long now = System.currentTimeMillis();

        if (yEdge && ejectState == EjectState.IDLE && hasAnyBall()) {
            // Start eject window
            ejectState = EjectState.EJECTING;
            ejecting = true;
            ejectEndTimeMs = now + 500;  // 0.5s "ejecting" window; adjust as you like
        }

        if (ejectState == EjectState.EJECTING && now >= ejectEndTimeMs) {
            ejectState = EjectState.IDLE;
            ejecting = false;
        }

        // You can expand this later to do pattern-based slot selection and auto-loading.
        // For now, we just return whether all 3 slots are marked as non-empty.
        return isFull();
    }

    // =========================
    // PERIODIC (call every loop)
    // =========================
    public void periodic() {
        applyPidfIfChanged();
        updateMotion();
        updateHold();   // active hold while idle

        // For tuning: DON'T constantly re-zero from the ABS encoder.
        // We'll only correct (optionally) in updateMotion at the end of a move.
        // if (!isMoving() && !pendingAutoRotate && !manualIntakeQueued) {
        //     verifyAndCorrectFromAbs();
        // }

        updateAutoRotateState();
    }


    // =========================
    // ACTIVE HOLD
    // =========================
    private void updateHold() {
        if (!SpindexerTuningConfig_State.HOLD_ENABLED) return;
        if (motionState != MotionState.IDLE) return;  // don't fight moves
        if (pendingAutoRotate || manualIntakeQueued) return;

        int target = shortestTicksToAngle(holdAngleDeg);
        int pos = motor.getCurrentPosition();
        int err = target - pos;

        if (Math.abs(err) <= SpindexerTuningConfig_State.HOLD_DEADBAND_TICKS) {
            // close enough -> don't buzz
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return;
        }

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SpindexerTuningConfig_State.HOLD_POWER);
    }

    // =========================
    // Auto rotate (for intake behavior)
    // =========================
    private void updateAutoRotateState() {
        if (!pendingAutoRotate) return;

        long now = System.currentTimeMillis();
        if (now < autoRotateTimeMs) return;
        if (motionState != MotionState.IDLE) return;

        pendingAutoRotate = false;
        int nextIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);

        autoRotateInProgress = requestMoveToAngle(
                slotCenterAngleAtIntake(nextIndex),
                SpindexerTuningConfig_State.MOVE_POWER,
                SpindexerTuningConfig_State.MOVE_TIMEOUT_MS,
                nextIndex
        );
    }

    // =========================
    // Manual intake helper (kept for your main teleop)
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
    // Motion core (non-blocking)
    // =========================
    private boolean requestMoveToAngle(double angleDeg, double power, long timeoutMs, int intakeIndexOnFinish) {
        if (motionState == MotionState.MOVING) return false;

        int target = shortestTicksToAngle(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        holdAngleDeg = angleDeg;  // ← This is already here and correct

        motionState = MotionState.MOVING;
        motionDeadlineMs = System.currentTimeMillis() + timeoutMs;
        pendingIntakeIndexOnFinish = intakeIndexOnFinish;

        return true;
    }

    private void updateMotion() {
        if (motionState != MotionState.MOVING) return;

        long now = System.currentTimeMillis();
        boolean timedOut = now >= motionDeadlineMs;

        if (!motor.isBusy() || timedOut) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motionState = MotionState.IDLE;

            if (pendingIntakeIndexOnFinish >= 0) {
                intakeIndex = pendingIntakeIndexOnFinish;
            }
            pendingIntakeIndexOnFinish = -1;

            autoRotateInProgress = false;

            // Do ABS correction if needed
            verifyAndCorrectFromAbs();

            // DON'T update holdAngleDeg here - it's already set to the target angle
            // DELETE THIS LINE:
            // holdAngleDeg = getCurrentAngleDeg();
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

        int diff = baseTicks - current;
        int k = (int) Math.round((double) diff / revTicks);

        return baseTicks - k * revTicks;
    }

    private void verifyAndCorrectFromAbs() {
        double absRaw = getAbsAngleDeg();
        double absInternal = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);
        double encInternal = getCurrentAngleDeg();
        double diff = smallestAngleDiff(encInternal, absInternal);

        if (Math.abs(diff) > ABS_REZERO_THRESHOLD_DEG) {
            autoZeroFromAbs();
        }
    }

    // =========================
    // Color helpers (kept for your main teleop)
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
    }

    // =========================
    // Apply PIDF from Panels (only when changed)
    // =========================
    private void applyPidfIfChanged() {
        // --- Position PIDF for RUN_TO_POSITION ---
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

        // --- Velocity PIDF for RUN_USING_ENCODER (optional, mostly for flywheels) ---
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
    /**
     * Returns desired shot color order for each tag.
     * You can change these to match your actual game patterns.
     */
    private Ball[] getPatternForTag(int tag) {
        switch (tag) {
            case 21: // example pattern
                return new Ball[]{ Ball.GREEN, Ball.PURPLE, Ball.GREEN };
            case 22:
                return new Ball[]{ Ball.PURPLE, Ball.GREEN, Ball.PURPLE };
            case 23:
                return new Ball[]{ Ball.GREEN, Ball.GREEN, Ball.PURPLE };
            default:
                return null; // "fast" mode / no pattern
        }
    }
}

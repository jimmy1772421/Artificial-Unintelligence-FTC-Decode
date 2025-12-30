package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpindexerSubsystem_State {

    public enum Ball {
        EMPTY,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    // Internal helper classification
    private enum RawColor {
        RED,
        GREEN,
        PURPLE
    }

    // =========================
    // MOTOR / GEOMETRY CONSTANTS
    // =========================
    private static final double TICKS_PER_REV = 383.6;
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;   // "home" angle
    private static final double LOAD_ANGLE   = 180.0;
    private static final int SLOT_COUNT = 3;
    private static final int TOLERANCE_TICKS = 10;
    private static final double MOVE_POWER = 0.5;

    // Analog abs encoder
    private static final double ABS_VREF = 3.3; // REV analog reference
    private static final double ABS_MECH_OFFSET_DEG = 245.0; // maybe add 2-5 deg

    private static final double ABS_REZERO_THRESHOLD_DEG = 3.0;

    // Motion timing
    private static final long MOVE_TIMEOUT_MS = 700;

    // If mag was full when started shooting
    private static final long WAIT_BEFORE_LOADER_FULL_MS    = 200;
    // If mag was NOT full when started shooting (give shooter more spin-up time)
    private static final long WAIT_BEFORE_LOADER_PARTIAL_MS = 2000;
    private static final long WAIT_AFTER_LOADER_MS  = 400;

    // =========================
    // HARDWARE
    // =========================
    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    // encoder value for angle 0° (intake) when slot 0 is at intake
    private int zeroTicks = 0;

    // which slot index is currently at intake (0,1,2)
    private int intakeIndex = 0;

    private final Ball[] slots = new Ball[SLOT_COUNT];

    // =========================
    // AUTO-INTAKE STATE
    // =========================
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;

    // Manual intake request (non-blocking version of intakeOne())
    private boolean manualIntakeQueued = false;
    private int manualIntakeQueuedTag = 0;

    // =========================
    // APRILTAG / PATTERN STATE
    // =========================
    // 23 = P, P, G
    // 22 = P, G, P
    // 21 = G, P, P
    //  0 = no valid tag -> fastest possible logic
    private int gameTag = 0;

    // =========================
    // NON-BLOCKING MOTION STATE MACHINE
    // =========================
    private enum MotionState { IDLE, MOVING }
    private MotionState motionState = MotionState.IDLE;
    private long motionDeadlineMs = 0;
    private int pendingIntakeIndexOnFinish = -1; // update intakeIndex when move ends (intake moves only)

    // =========================
    // NON-BLOCKING EJECT STATE MACHINE
    // =========================
    private enum EjectState {
        IDLE,
        SELECT_AND_MOVE_TO_LOAD,
        MOVING_TO_LOAD,
        WAIT_BEFORE_LOADER,
        WAIT_AFTER_LOADER,
        HOMING
    }

    private boolean ejecting = false;
    private EjectState ejectState = EjectState.IDLE;
    private int ejectSlotIndex = 0;
    private long ejectPhaseTimeMs = 0;

    private int patternStep = 0;
    private boolean startedWithFullMag = false;
    private boolean firstShotDone = false;

    private double lastPosP = Double.NaN;
    private PIDFCoefficients lastVel = new PIDFCoefficients(0,0,0,0);


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

        // One-shot auto-zero at startup using absolute encoder
        autoZeroFromAbs();

        // Non-blocking home request (will complete as update() runs)
        homeToIntake();
    }

    // =========================
    // PUBLIC TAG API
    // =========================
    public void setGameTag(int tag) { this.gameTag = tag; }
    public int getGameTag() { return gameTag; }

    // =========================
    // ABS ENCODER HELPERS
    // =========================
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

    // =========================
    // ANGLE / TICKS HELPERS
    // =========================
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
        int baseTicks = zeroTicks + (int)Math.round(desiredRevs * TICKS_PER_REV);

        int current = motor.getCurrentPosition();
        int revTicks = (int)Math.round(TICKS_PER_REV);

        int diff = baseTicks - current;
        int k = (int)Math.round((double)diff / revTicks);

        return baseTicks - k * revTicks;
    }

    // =========================
    // NON-BLOCKING MOTION CORE
    // =========================
    private boolean requestMoveToAngle(double angleDeg, double power, long timeoutMs, int intakeIndexOnFinish) {
        if (motionState == MotionState.MOVING) return false;

        int target = shortestTicksToAngle(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

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

            // Re-sync encoder model to abs encoder after any move (same as your old behavior)
            verifyAndCorrectFromAbs();
        }
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private boolean requestMoveSlotToIntake(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex);
        return requestMoveToAngle(angle, power, MOVE_TIMEOUT_MS, slotIndex);
    }

    private boolean requestMoveSlotToLoad(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        return requestMoveToAngle(angle, power, MOVE_TIMEOUT_MS, -1);
    }

    // =========================
    // COLOR / BALL HANDLING
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
        if (sum < 50) {
            return RawColor.RED;
        }

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        boolean greenDominant =
                (gn > rn + 0.05) &&
                        (gn > bn + 0.02);

        boolean redDominant =
                (rn > gn + 0.10) &&
                        (rn > bn + 0.05);

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

        if (!present1 && !present2) {
            return Ball.EMPTY;
        }

        RevColorSensorV3 chosenSensor;
        if (present1 && present2) chosenSensor = (d1 <= d2) ? intakeColor : intakeColor2;
        else if (present1) chosenSensor = intakeColor;
        else chosenSensor = intakeColor2;

        RawColor raw = classifyColor(chosenSensor);

        switch (raw) {
            case GREEN:  return Ball.GREEN;
            case PURPLE: return Ball.PURPLE;
            case RED:
            default:     return Ball.EMPTY;
        }
    }

    private boolean isIntakeSlotAtIntakePosition() {
        double desiredAngle = slotCenterAngleAtIntake(intakeIndex);
        double currentAngle = getCurrentAngleDeg();

        double diff = smallestAngleDiff(currentAngle, desiredAngle);
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;
        return Math.abs(diff) <= tolDeg;
    }

    // =========================
    // PATTERN HELPERS
    // =========================
    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];

        switch (tag) {
            case 23:
                seq[0] = Ball.PURPLE; seq[1] = Ball.PURPLE; seq[2] = Ball.GREEN;
                return seq;
            case 22:
                seq[0] = Ball.PURPLE; seq[1] = Ball.GREEN;  seq[2] = Ball.PURPLE;
                return seq;
            case 21:
                seq[0] = Ball.GREEN;  seq[1] = Ball.PURPLE; seq[2] = Ball.PURPLE;
                return seq;
            default:
                return null;
        }
    }

    private int selectFastestNonEmptySlot(double currentAngle) {
        int bestSlot = -1;
        double bestDiff = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (!slotHasBall(i)) continue;

            double targetAngle = slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
            double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

            if (diff < bestDiff) {
                bestDiff = diff;
                bestSlot = i;
            }
        }
        return bestSlot;
    }

    private int selectNextEjectSlotIndex() {
        double currentAngle = getCurrentAngleDeg();

        Ball[] pattern = getPatternForTag(gameTag);
        boolean usePattern = (pattern != null && patternStep < pattern.length);

        if (usePattern) {
            Ball desired = pattern[patternStep];

            int bestSlot = -1;
            double bestDiff = Double.MAX_VALUE;

            for (int i = 0; i < SLOT_COUNT; i++) {
                if (slots[i] != desired) continue;

                double targetAngle = slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
                double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

                if (diff < bestDiff) {
                    bestDiff = diff;
                    bestSlot = i;
                }
            }

            if (bestSlot != -1) return bestSlot;
        }

        return selectFastestNonEmptySlot(currentAngle);
    }

    // =========================
    // MANUAL "intakeOne" (NOW NON-BLOCKING)
    // =========================
    public void intakeOne(Telemetry telemetry, int tag) {
        this.gameTag = tag;
        manualIntakeQueued = true;
        manualIntakeQueuedTag = tag;

        telemetry.addData("ManualIntake", "Queued intakeOne(tag=%d)", tag);
    }

    // =========================
    // MAIN UPDATE (CALL EVERY LOOP)
    // =========================
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        // 1) Always advance motion first (this replaces all blocking waits)
        updateMotion();

        // --- Optional manual override for gameTag from driver ---
        if (patternTagOverride == 21 ||
                patternTagOverride == 22 ||
                patternTagOverride == 23 ||
                patternTagOverride == 0) {
            gameTag = patternTagOverride;
        }

        // --- Handle eject button press ---
        if (yEdge && !ejecting) {
            // Cancel pending intake actions when eject begins
            pendingAutoRotate = false;
            manualIntakeQueued = false;

            if (hasAnyBall()) {
                startedWithFullMag = isFull();
            } else {
                // SPECIAL CASE: fake 3 shots
                Ball[] pattern = getPatternForTag(gameTag);
                if (pattern != null) {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = pattern[i];
                } else {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.PURPLE;
                }
                startedWithFullMag = false; // force longer first-shot delay
            }

            ejecting = true;
            ejectState = EjectState.SELECT_AND_MOVE_TO_LOAD;
            patternStep = 0;
            firstShotDone = false;
        }

        // =========================
        // MANUAL INTAKE (queued) - non-blocking
        // =========================
        if (manualIntakeQueued && !ejecting) {
            // Ensure we're aligned to intake before sampling
            if (motionState == MotionState.IDLE && !isIntakeSlotAtIntakePosition()) {
                requestMoveSlotToIntake(intakeIndex, MOVE_POWER);
            } else if (motionState == MotionState.IDLE && isIntakeSlotAtIntakePosition()) {
                Ball color = detectBallColor();
                slots[intakeIndex] = color;

                telemetry.addData("ManualIntake",
                        "Slot %d = %s (tag=%d)", intakeIndex, color, manualIntakeQueuedTag);

                pendingAutoRotate = true;
                autoRotateTimeMs = System.currentTimeMillis() + 100;

                manualIntakeQueued = false;
            }
        }

        // =========================
        // AUTO INTAKE (only when stable at intake)
        // =========================
        if (!isFull()
                && !ejecting
                && !manualIntakeQueued
                && motionState == MotionState.IDLE
                && isIntakeSlotAtIntakePosition()) {

            boolean ballPresent = isBallPresent();

            if (ballPresent && !lastBallPresent && slots[intakeIndex] == Ball.EMPTY) {
                Ball color = detectBallColor();

                if (color == Ball.GREEN || color == Ball.PURPLE) {
                    slots[intakeIndex] = color;
                    telemetry.addData("AutoIntake", "Slot %d = %s", intakeIndex, color);

                    pendingAutoRotate = true;
                    autoRotateTimeMs = System.currentTimeMillis() + 100;
                } else {
                    telemetry.addData("AutoIntake", "Ignored non-ball at slot %d", intakeIndex);
                }
            }

            lastBallPresent = ballPresent;
        }

        // =========================
        // Pending auto-rotate (now uses motion state machine)
        // =========================
        if (pendingAutoRotate
                && !ejecting
                && !manualIntakeQueued
                && motionState == MotionState.IDLE
                && System.currentTimeMillis() >= autoRotateTimeMs) {

            pendingAutoRotate = false;
            int nextIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
            requestMoveSlotToIntake(nextIndex, MOVE_POWER);
        }

        // =========================
        // EJECT STATE MACHINE (no blocking moves)
        // =========================
        if (ejecting) {
            long now = System.currentTimeMillis();

            switch (ejectState) {
                case SELECT_AND_MOVE_TO_LOAD: {
                    if (!hasAnyBall()) {
                        homeToIntake();
                        ejectState = EjectState.HOMING;
                        break;
                    }

                    if (motionState != MotionState.IDLE) break;

                    int nextSlot = selectNextEjectSlotIndex();
                    if (nextSlot < 0 || nextSlot >= SLOT_COUNT) {
                        homeToIntake();
                        ejectState = EjectState.HOMING;
                        break;
                    }

                    ejectSlotIndex = nextSlot;

                    boolean started = requestMoveSlotToLoad(ejectSlotIndex, MOVE_POWER);
                    if (started) {
                        ejectState = EjectState.MOVING_TO_LOAD;
                    }
                    break;
                }

                case MOVING_TO_LOAD: {
                    if (motionState != MotionState.IDLE) break;

                    // Only use the long delay for the FIRST shot when we started partial
                    long delayMs;
                    if (!firstShotDone && !startedWithFullMag) delayMs = WAIT_BEFORE_LOADER_PARTIAL_MS;
                    else delayMs = WAIT_BEFORE_LOADER_FULL_MS;

                    ejectPhaseTimeMs = now + delayMs;
                    ejectState = EjectState.WAIT_BEFORE_LOADER;
                    break;
                }

                case WAIT_BEFORE_LOADER: {
                    if (now >= ejectPhaseTimeMs) {
                        loader.startCycle();
                        firstShotDone = true;
                        ejectPhaseTimeMs = now + WAIT_AFTER_LOADER_MS;
                        ejectState = EjectState.WAIT_AFTER_LOADER;
                    }
                    break;
                }

                case WAIT_AFTER_LOADER: {
                    if (now >= ejectPhaseTimeMs) {
                        clearSlot(ejectSlotIndex);

                        Ball[] pattern = getPatternForTag(gameTag);
                        boolean usingPattern = (pattern != null);
                        if (usingPattern) patternStep++;

                        if (!hasAnyBall() || (usingPattern && patternStep >= pattern.length)) {
                            homeToIntake();
                            ejectState = EjectState.HOMING;
                        } else {
                            ejectState = EjectState.SELECT_AND_MOVE_TO_LOAD;
                        }
                    }
                    break;
                }

                case HOMING: {
                    if (motionState == MotionState.IDLE) {
                        ejecting = false;
                        startedWithFullMag = false;
                        firstShotDone = false;
                        ejectState = EjectState.IDLE;
                    }
                    break;
                }

                default:
                case IDLE:
                    break;
            }
        }

        // Abs correction when totally idle (same intent as your old code)
        if (motionState == MotionState.IDLE && !ejecting && !pendingAutoRotate && !manualIntakeQueued) {
            verifyAndCorrectFromAbs();
        }

        return isFull();
    }

    // =========================
    // SLOT HELPERS / GETTERS
    // =========================
    public Ball[] getSlots() { return slots; }
    public int getEncoder() { return motor.getCurrentPosition(); }
    public int getTarget() { return motor.getTargetPosition(); }
    public int getIntakeIndex() { return intakeIndex; }
    public boolean isEjecting() { return ejecting; }
    public boolean isMoving() { return motionState != MotionState.IDLE; }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public boolean slotHasBall(int slotIndex) {
        return slots[slotIndex] != Ball.EMPTY;
    }

    public void clearSlot(int slotIndex) {
        slots[slotIndex] = Ball.EMPTY;
    }

    // =========================
    // NON-BLOCKING HOME
    // =========================
    public void homeToIntake() {
        // request move (non-blocking)
        requestMoveSlotToIntake(0, MOVE_POWER);
    }

    // =========================
    // ABS VERIFY / CORRECT
    // =========================
    private void verifyAndCorrectFromAbs() {
        double absRaw = getAbsAngleDeg();
        double absInternal = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);
        double encInternal = getCurrentAngleDeg();

        double diff = smallestAngleDiff(encInternal, absInternal);

        if (Math.abs(diff) > ABS_REZERO_THRESHOLD_DEG) {
            autoZeroFromAbs();
        }
    }

    //    FORCE SLOTS

    public void forceIntakeSlotGreen(Telemetry telemetry) {
        forceIntakeSlotColor(telemetry, Ball.GREEN);
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        forceIntakeSlotColor(telemetry, Ball.PURPLE);
    }

    private void forceIntakeSlotColor(Telemetry telemetry, Ball forcedColor) {
        // Only overwrite if this slot is currently empty
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = forcedColor;

            if (telemetry != null) {
                telemetry.addData("ForceIntake", "Slot %d forced to %s", intakeIndex, forcedColor);
            }

            // mimic normal auto-intake behavior: schedule auto-rotate
            pendingAutoRotate = true;
            autoRotateTimeMs = System.currentTimeMillis() + 100;
        } else {
            if (telemetry != null) {
                telemetry.addData("ForceIntake", "Slot %d not empty (=%s), ignore force %s",
                        intakeIndex, slots[intakeIndex], forcedColor);
            }
        }
    }


    // =========================
    // DEBUG
    // =========================
    public void debugAbsAngle(Telemetry telemetry) {
        double raw = getAbsAngleDeg();
        double corrected = normalizeAngle(raw - ABS_MECH_OFFSET_DEG);
        double encAngle = getCurrentAngleDeg();
        telemetry.addData("Abs raw", "%.1f deg", raw);
        telemetry.addData("Abs corr (slot0@intake=0)", "%.1f deg", corrected);
        telemetry.addData("Enc angle", "%.1f deg", encAngle);
        telemetry.addData("Abs offset", "%.1f deg", ABS_MECH_OFFSET_DEG);
        telemetry.addData("Intake slot index", intakeIndex);
    }

    public void applyPidfIfChanged() {
        // Position loop (RUN_TO_POSITION): only P used
        if (SpindexerTuningConfig_State.POS_P != lastPosP) {
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_TO_POSITION,
                    new PIDFCoefficients(SpindexerTuningConfig_State.POS_P, 0, 0, 0)
            );
            lastPosP = SpindexerTuningConfig_State.POS_P;
        }

        // Velocity loop (RUN_USING_ENCODER)
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

    public boolean cmdIntakeSlot(int slot) {
        slot = ((slot % 3) + 3) % 3;
        return requestMoveSlotToIntake(slot, SpindexerTuningConfig_State.MOVE_POWER);
    }

    public boolean cmdLoadSlot(int slot) {
        slot = ((slot % 3) + 3) % 3;
        return requestMoveSlotToLoad(slot, SpindexerTuningConfig_State.MOVE_POWER);
    }

    public boolean cmdAngle(double angleDeg) {
        return requestMoveToAngle(angleDeg, SpindexerTuningConfig_State.MOVE_POWER,
                SpindexerTuningConfig_State.MOVE_TIMEOUT_MS, -1);
    }

    // Call this every loop in the tuner (and in your real teleop too)
    public void periodic() {
        applyPidfIfChanged();
        updateMotion(); // your non-blocking motion stepper
    }

//    public boolean isMoving() {
//        return motionState != MotionState.IDLE; // if you use this enum
//    }
    public int getIntakeSlotIndex() {
        return intakeIndex; // <-- change if your field name differs
    }

    // If you already have some angle getter, return it here.
// Otherwise compute it (this assumes you have motor/zeroTicks/TICKS_PER_REV and normalizeAngle()).
//    public double getCurrentAngleDeg() {
//        int ticks = motor.getCurrentPosition();  // <-- change motor name if different
//        int relTicks = ticks - zeroTicks;        // <-- change zeroTicks name if different
//        double revs = relTicks / TICKS_PER_REV;  // <-- change TICKS_PER_REV if different
//        double angle = revs * 360.0;
//        return normalizeAngle(angle);            // <-- if you don't have normalizeAngle(), I’ll give you one
//    }


    public String getGamePattern() {
        Ball[] pattern = getPatternForTag(gameTag); // <-- if your helper/field names differ, rename them
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

    // True if we have an auto-rotate queued OR currently rotating due to that queue
    public boolean isAutoRotating() {
        // If you have a simple pending flag, this is enough:
        return pendingAutoRotate;

        // If you want it to also report "currently moving due to auto rotate",
        // you can expand it like this (uncomment if you have motionState):
        // return pendingAutoRotate || (motionState != MotionState.IDLE);
    }


}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_new;

public class SpindexerSubsystem_State_new {

    // ==== BALL TYPES ====

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

    // High-level state for eject sequence
    private enum EjectState {
        IDLE,
        ROTATING_TO_LOAD,
        WAIT_BEFORE_LOADER,
        WAIT_AFTER_LOADER
    }

    // ==== MOTOR / GEOMETRY CONSTANTS ====

    // goBILDA 435 rpm YJ integrated encoder
    private static final double TICKS_PER_REV = 383.6;
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;   // "home" angle for slot 0
    private static final double LOAD_ANGLE   = 180.0; // angle offset between intake and load
    private static final int SLOT_COUNT = 3;
    private static final int TOLERANCE_TICKS = 4;

    // Max power output used by the PID loop
    private static final double MAX_POWER = 0.5;

    // Analog abs encoder
    private static final double ABS_VREF = 3.3; // ELC analog reference

    // Raw abs angle (deg) when SLOT 0 is perfectly at intake
    // i.e. absRaw == 245°  <=>  internal angle == 0°
    private static final double ABS_MECH_OFFSET_DEG = (286.5-180); // maybe add 2-5 deg

    private static final double ABS_REZERO_THRESHOLD_DEG = 3.0;

    // If mag was full when started shooting
    private static final long WAIT_BEFORE_LOADER_FULL_MS    = 500;
    // If mag was NOT full when started shooting (give shooter more spin-up time)
    private static final long WAIT_BEFORE_LOADER_PARTIAL_MS = 2000;
    private static final long WAIT_AFTER_LOADER_MS          = 400;

    // ==== HARDWARE ====

    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    // ==== ANGLE / ENCODER STATE ====

    // encoder value for angle 0° (intake) when slot 0 is at intake
    private int zeroTicks = 0;

    // which slot index is currently at intake (0,1,2)
    private int intakeIndex = 0;

    // Requested target angle in the internal frame (slot0@intake = 0°)
    private double targetAngleDeg = 0.0;

    // Manual PIDF for angle control (tuned via setPIDF from your tuner)
    private PIDFCoefficients anglePIDF = new PIDFCoefficients(0, 0, 0, 0);
    private double angleIntegral = 0.0;
    private double lastAngleErrorDeg = 0.0;
    private long lastAngleUpdateTimeNs = System.nanoTime();

    // ==== SLOT / BALL STATE ====

    private final Ball[] slots = new Ball[SLOT_COUNT];

    // Auto-intake state
    private boolean lastBallPresent = false;

    private enum AutoIntakeState {
        IDLE,
        WAIT_FOR_ROTATE,
        ROTATING
    }

    private AutoIntakeState autoIntakeState = AutoIntakeState.IDLE;
    private int autoIntakeNextSlotIndex = 0;
    private long autoRotateTimeMs = 0;


    // Eject sequence state (state-based, non-blocking)
    private boolean ejecting = false;
    private EjectState ejectState = EjectState.IDLE;
    private int ejectSlotIndex = 0;
    private long ejectPhaseTime = 0;
    // pattern step when using AprilTag pattern
    private int patternStep = 0;
    // Whether the mag was full (3 balls) when this eject sequence started
    private boolean startedWithFullMag = false;

    // Eject start index used by isAtMid()
    private int ejectStartIndex = 0;

    // ==== AprilTag / pattern state ====
    // 23 = P, P, G
    // 22 = P, G, P
    // 21 = G, P, P
    //  0 = no valid tag -> fastest possible logic
    private int gameTag = 0;

    public SpindexerSubsystem_State_new(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");
        absEncoder = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
        }

        // One-shot auto-zero at startup using absolute encoder
        autoZeroFromAbs();

        // Start by holding the current intake slot at its intake angle
        targetAngleDeg = slotCenterAngleAtIntake(intakeIndex);
        lastAngleUpdateTimeNs = System.nanoTime();
    }

    // ==========================
    // ===== PIDF INTERFACE =====
    // ==========================

    /**
     * Called by your tuner (e.g. SpindexerTuningConfig_new) to set PIDF.
     */
    public void setPIDF(PIDFCoefficients pidf) {
        if (pidf != null) {
            this.anglePIDF = pidf;
        }
    }

    /**
     * Overload if your tuner passes scalars instead.
     */
    public void setPIDF(double p, double i, double d, double f) {
        this.anglePIDF = new PIDFCoefficients(p, i, d, f);
    }

    public PIDFCoefficients getPIDF() {
        return anglePIDF;
    }

    // ==============================
    // ===== ABS ENCODER HELPERS ====
    // ==============================

    private double getAbsAngleDeg() {
        // Converts 0–3.3V → 0–360°
        double v = absEncoder.getVoltage();
        double angle = (v / ABS_VREF) * 360.0;

        // Normalize to [0, 360)
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    /** normalize any angle to [0, 360) */
    private double normalizeAngle(double angleDeg) {
        return (angleDeg % 360.0 + 360.0) % 360.0;
    }

    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0; // wrap to [-180, 180)
        return diff;
    }

    /**
     * Use the absolute encoder to align the encoder ticks with the real mechanical angle.
     *
     * - We define internalAngle = 0° when absRaw == ABS_MECH_OFFSET_DEG (slot 0 at intake).
     * - internalAngle increases with rotation, 360° per rev.
     * - We solve for zeroTicks so that ticksToAngle(currentTicks) == internalAngle.
     * - We also estimate which slot is currently at intake.
     */
    private void autoZeroFromAbs() {
        double absRaw = getAbsAngleDeg();

        // internalAngle = 0 when absRaw == ABS_MECH_OFFSET_DEG
        double internalAngle = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);

        int currentTicks = motor.getCurrentPosition();
        int internalTicks = (int) Math.round((internalAngle / 360.0) * TICKS_PER_REV);

        // internalAngle = (currentTicks - zeroTicks) * 360 / TICKS_PER_REV
        // => zeroTicks = currentTicks - internalTicks
        zeroTicks = currentTicks - internalTicks;

        // Figure out which slot is closest to intake (internalAngle 0,120,240)
        double slotIndexF = internalAngle / DEGREES_PER_SLOT; // 0..3
        int nearestIndex = (int) Math.round(slotIndexF) % SLOT_COUNT;
        if (nearestIndex < 0) nearestIndex += SLOT_COUNT;
        intakeIndex = nearestIndex;

        // Reset PID integrator when we rezero
        angleIntegral = 0.0;
        lastAngleErrorDeg = 0.0;
    }

    // =========================
    // ===== ANGLE HELPERS =====
    // =========================

    private double ticksToAngle(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        double angle = revs * 360.0;
        return normalizeAngle(angle);
    }

    private int angleToTicks(double angleDeg) {
        double norm = normalizeAngle(angleDeg);
        double revs = norm / 360.0;
        return zeroTicks + (int) Math.round(revs * TICKS_PER_REV);
    }

    /**
     * Angle estimated from motor encoder, in degrees [0, 360).
     * 0° = slot 0 at intake (raw abs ≈ ABS_MECH_OFFSET_DEG).
     */
    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private double slotCenterAngleAtLoad(int slotIndex) {
        return slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
    }

    /**
     * Command an absolute internal angle in degrees. Non-blocking.
     */
    public void setTargetAngleDeg(double angleDeg) {
        this.targetAngleDeg = normalizeAngle(angleDeg);
    }

    /**
     * Command a slot to be at the intake position (non-blocking).
     */
    public void commandSlotToIntake(int slotIndex) {
        slotIndex = ((slotIndex % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        setTargetAngleDeg(slotCenterAngleAtIntake(slotIndex));
        intakeIndex = slotIndex;
    }

    /**
     * Command a slot to be at the load position (non-blocking).
     */
    public void commandSlotToLoad(int slotIndex) {
        slotIndex = ((slotIndex % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        setTargetAngleDeg(slotCenterAngleAtLoad(slotIndex));
    }

    private boolean isAngleNear(double currentDeg, double targetDeg) {
        double diff = smallestAngleDiff(currentDeg, targetDeg);
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;
        return Math.abs(diff) <= tolDeg;
    }

    private boolean isSlotAtIntakePosition(int slotIndex) {
        double desiredAngle = slotCenterAngleAtIntake(slotIndex);
        return isAngleNear(getCurrentAngleDeg(), desiredAngle);
    }

    private boolean isSlotAtLoadPosition(int slotIndex) {
        double desiredAngle = slotCenterAngleAtLoad(slotIndex);
        return isAngleNear(getCurrentAngleDeg(), desiredAngle);
    }

    /**
     * Core PIDF loop. Call once per cycle (from update()).
     */
    private void updateAnglePID() {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastAngleUpdateTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastAngleUpdateTimeNs = nowNs;

        double currentAngle = getCurrentAngleDeg();
        double errorDeg = smallestAngleDiff(targetAngleDeg, currentAngle);

        // Integrator with simple anti-windup
        angleIntegral += errorDeg * dt;
        double maxIntegral = 100.0; // tune if needed
        angleIntegral = Range.clip(angleIntegral, -maxIntegral, maxIntegral);

        double derivative = (errorDeg - lastAngleErrorDeg) / dt;
        lastAngleErrorDeg = errorDeg;

        double p = anglePIDF.p;
        double i = anglePIDF.i;
        double d = anglePIDF.d;
        double f = anglePIDF.f;

        // Feedforward: constant term in direction of error (to overcome friction)
        double ff = f * Math.signum(errorDeg);

        double output = p * errorDeg + i * angleIntegral + d * derivative + ff;
        double power = Range.clip(output, -MAX_POWER, MAX_POWER);

        motor.setPower(power);
    }

    /**
     * Compare encoder-based angle vs abs-encoder-based angle.
     * If they disagree by more than ABS_REZERO_THRESHOLD_DEG,
     * re-run autoZeroFromAbs() to snap zeroTicks + intakeIndex to the abs encoder.
     */
    private void verifyAndCorrectFromAbs() {
        // Abs angle in the same 0° frame as internal angle (slot0@intake = 0°)
        double absRaw = getAbsAngleDeg();
        double absInternal = normalizeAngle(absRaw - ABS_MECH_OFFSET_DEG);

        // Current internal angle based on motor encoder
        double encInternal = getCurrentAngleDeg();

        double diff = smallestAngleDiff(encInternal, absInternal);

        if (Math.abs(diff) > ABS_REZERO_THRESHOLD_DEG) {
            // We're out of sync → trust abs encoder and recompute zeroTicks + intakeIndex
            autoZeroFromAbs();
            // After rezero, hold whatever intake slot autoZeroFromAbs decided
            targetAngleDeg = slotCenterAngleAtIntake(intakeIndex);
        }
    }

    // =========================
    // ===== COLOR / BALLS =====
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
            // Very dark / noisy → treat as RED-ish junk so it gets ignored
            return RawColor.RED;
        }

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        // Green-dominant: green noticeably higher than both red & blue
        boolean greenDominant =
                (gn > rn + 0.05) &&
                        (gn > bn + 0.02);

        // Red-dominant: red clearly higher than both g & b (your red arms)
        boolean redDominant =
                (rn > gn + 0.10) &&
                        (rn > bn + 0.05);

        if (greenDominant) {
            return RawColor.GREEN;
        }
        if (redDominant) {
            return RawColor.RED;
        }

        // Everything else → PURPLE (blue-ish game ball)
        return RawColor.PURPLE;
    }

    private Ball detectBallColor() {
        final double THRESH_CM = 5.0;

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        if (!present1 && !present2) {
            // Nothing close enough to be a ball
            return Ball.EMPTY;
        }

        // Choose which sensor to trust for color
        RevColorSensorV3 chosenSensor;
        if (present1 && present2) {
            // Use whichever is closer
            chosenSensor = (d1 <= d2) ? intakeColor : intakeColor2;
        } else if (present1) {
            chosenSensor = intakeColor;
        } else {
            chosenSensor = intakeColor2;
        }

        RawColor raw = classifyColor(chosenSensor);

        switch (raw) {
            case GREEN:
                return Ball.GREEN;
            case PURPLE:
                return Ball.PURPLE;
            case RED:
            default:
                // RED == arm / junk → treat as not-a-ball
                return Ball.EMPTY;
        }
    }

    // ================================
    // ===== TAG / PATTERN HELPERS ====
    // ================================

    public void setGameTag(int tag) {
        this.gameTag = tag;
    }

    public int getGameTag() {
        return gameTag;
    }

    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];

        switch (tag) {
            case 23: // purple, purple, green
                seq[0] = Ball.PURPLE;
                seq[1] = Ball.PURPLE;
                seq[2] = Ball.GREEN;
                return seq;

            case 22: // purple, green, purple
                seq[0] = Ball.PURPLE;
                seq[1] = Ball.GREEN;
                seq[2] = Ball.PURPLE;
                return seq;

            case 21: // green, purple, purple
                seq[0] = Ball.GREEN;
                seq[1] = Ball.PURPLE;
                seq[2] = Ball.PURPLE;
                return seq;

            default:
                // 0 or anything else = "no valid tag" (no pattern)
                return null;
        }
    }

    /**
     * Fastest non-empty slot to rotate to LOAD, ignoring color.
     */
    private int selectFastestNonEmptySlot(double currentAngle) {
        int bestSlot = -1;
        double bestDiff = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (!slotHasBall(i)) continue;

            double targetAngle = slotCenterAngleAtLoad(i);
            double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

            if (diff < bestDiff) {
                bestDiff = diff;
                bestSlot = i;
            }
        }

        return bestSlot;
    }

    /**
     * Choose which slot to eject next, using tag pattern when applicable.
     */
    private int selectNextEjectSlotIndex() {
        double currentAngle = getCurrentAngleDeg();

        Ball[] pattern = getPatternForTag(gameTag);
        boolean usePattern = (pattern != null && patternStep < pattern.length);

        if (usePattern) {
            Ball desired = pattern[patternStep];

            // 1st pass: nearest slot of the desired color
            int bestSlot = -1;
            double bestDiff = Double.MAX_VALUE;

            for (int i = 0; i < SLOT_COUNT; i++) {
                if (slots[i] != desired) continue;

                double targetAngle = slotCenterAngleAtLoad(i);
                double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

                if (diff < bestDiff) {
                    bestDiff = diff;
                    bestSlot = i;
                }
            }

            if (bestSlot != -1) {
                return bestSlot;
            }
            // If we didn't find the desired color, we just fall through to fastest non-empty
        }

        // Either no pattern, pattern finished, or we don't have that color available
        return selectFastestNonEmptySlot(currentAngle);
    }

    // ================================
    // ===== MAIN UPDATE (STATE) =====
    // ================================

    /**
     * Main non-blocking update: auto-intake + ejection + PID.
     *
     * Call this **every loop** from TeleOp/Auto.
     *
     * @param telemetry            telemetry for debug
     * @param loader               loader subsystem
     * @param yEdge                true on *rising edge* of shoot button (e.g. Y)
     * @param patternTagOverride   driver override for tag: 23/22/21/0 allowed
     * @return true if mag is full after this update
     */
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        long now = System.currentTimeMillis();

        // === 1. Sync PIDF gains from Panels every loop ===
        setPIDF(
                SpindexerTuningConfig_new.kP,
                SpindexerTuningConfig_new.kI,
                SpindexerTuningConfig_new.kD,
                SpindexerTuningConfig_new.kF
        );

        // --- Optional: allow Panels to override the tag too ---
        // Start with whatever the code passed in
        int effectiveTagOverride = patternTagOverride;

        // If Panels' config has a valid tag, let it win
        int cfgTag = SpindexerTuningConfig_new.patternTagOverride;
        if (cfgTag == 0 || cfgTag == 21 || cfgTag == 22 || cfgTag == 23) {
            effectiveTagOverride = cfgTag;
        }

        // --- Optional manual override for gameTag from driver / Panels ---
        if (effectiveTagOverride == 21 ||
                effectiveTagOverride == 22 ||
                effectiveTagOverride == 23 ||
                effectiveTagOverride == 0) {
            gameTag = effectiveTagOverride;
        }


        // --- Handle eject button press (start eject sequence) ---
        if (yEdge && !ejecting) {
            if (hasAnyBall()) {
                // Normal case: we have tracked balls, shoot according to slots[] + pattern
                ejecting = true;
                ejectState = EjectState.ROTATING_TO_LOAD;
                patternStep = 0;
                startedWithFullMag = isFull();   // true if we had 3 balls when we started

            } else {
                // SPECIAL CASE:
                // No balls *registered* in slots[], but driver wants to
                // "try to shoot all three" again (in case balls are stuck / misread).

                Ball[] pattern = getPatternForTag(gameTag);
                if (pattern != null) {
                    // Use pattern colors
                    for (int i = 0; i < SLOT_COUNT; i++) {
                        slots[i] = pattern[i];
                    }
                } else {
                    // No pattern (tag = 0) → just pretend all three are PURPLE
                    for (int i = 0; i < SLOT_COUNT; i++) {
                        slots[i] = Ball.PURPLE;
                    }
                }

                ejecting = true;
                ejectState = EjectState.ROTATING_TO_LOAD;
                patternStep = 0;
                startedWithFullMag = false; // first shot uses longer delay
            }
        }

        // --- AUTO INTAKE STATE MACHINE ---
        boolean ballPresent = isBallPresent();

        switch (autoIntakeState) {
            case IDLE:
                // Look for a new ball in the current intake slot
                if (!isFull()
                        && ballPresent
                        && slots[intakeIndex] == Ball.EMPTY
                        && !ejecting) {

                    Ball color = detectBallColor();

                    // Only save real balls (GREEN / PURPLE). RED got filtered out earlier.
                    if (color == Ball.GREEN || color == Ball.PURPLE) {
                        slots[intakeIndex] = color;
                        telemetry.addData("AutoIntake", "Slot %d = %s", intakeIndex, color);

                        // Plan to rotate to the next slot after a short delay
                        autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
                        autoRotateTimeMs = now + 100;
                        autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
                    } else {
                        telemetry.addData("AutoIntake", "Ignored non-ball at slot %d", intakeIndex);
                    }
                }
                break;

            case WAIT_FOR_ROTATE:
                // Wait out the small delay before starting rotation
                if (now >= autoRotateTimeMs && !ejecting && !isFull()) {
                    // Start rotating toward the next intake slot
                    setTargetAngleDeg(slotCenterAngleAtIntake(autoIntakeNextSlotIndex));
                    autoIntakeState = AutoIntakeState.ROTATING;
                }
                break;

            case ROTATING:
                // Wait until we actually reach the new slot at intake
                if (isSlotAtIntakePosition(autoIntakeNextSlotIndex)) {
                    // Now we *know* this slot is at the intake
                    intakeIndex = autoIntakeNextSlotIndex;
                    autoIntakeState = AutoIntakeState.IDLE;
                }
                break;
        }

        lastBallPresent = ballPresent;


        // --- Eject sequence STATE MACHINE (non-blocking) ---
        if (ejecting) {
            switch (ejectState) {
                case ROTATING_TO_LOAD:
                    if (!hasAnyBall()) {
                        // Nothing left to shoot in our internal model
                        homeToIntake();
                        ejecting = false;
                        startedWithFullMag = false;
                        ejectState = EjectState.IDLE;
                        break;
                    }

                    // If we just entered this state, pick which slot to shoot
                    // (Whenever we re-enter ROTATING_TO_LOAD, we pick again)
                    ejectSlotIndex = selectNextEjectSlotIndex();
                    if (ejectSlotIndex < 0 || ejectSlotIndex >= SLOT_COUNT) {
                        homeToIntake();
                        ejecting = false;
                        startedWithFullMag = false;
                        ejectState = EjectState.IDLE;
                        break;
                    }

                    // Command that slot to LOAD
                    commandSlotToLoad(ejectSlotIndex);

                    // Wait until the slot is actually at LOAD
                    if (isSlotAtLoadPosition(ejectSlotIndex)) {
                        long delayMs = startedWithFullMag
                                ? WAIT_BEFORE_LOADER_FULL_MS
                                : WAIT_BEFORE_LOADER_PARTIAL_MS;

                        ejectPhaseTime = now + delayMs;
                        ejectState = EjectState.WAIT_BEFORE_LOADER;

                        // After the first shot, treat as "full" for subsequent ones (short delay)
                        startedWithFullMag = true;
                    }
                    break;

                case WAIT_BEFORE_LOADER:
                    if (now >= ejectPhaseTime) {
                        // In PIDF tuning TeleOp we may pass loader == null.
                        if (loader != null) {
                            loader.startCycle();
                        }
                        ejectPhaseTime = now + WAIT_AFTER_LOADER_MS;
                        ejectState = EjectState.WAIT_AFTER_LOADER;
                    }
                    break;


                case WAIT_AFTER_LOADER:
                    if (now >= ejectPhaseTime) {
                        clearSlot(ejectSlotIndex);

                        Ball[] pattern = getPatternForTag(gameTag);
                        if (pattern != null) {
                            patternStep++;
                        }
                        boolean usingPattern = (pattern != null);

                        if (!hasAnyBall() ||
                                (usingPattern && patternStep >= pattern.length)) {

                            homeToIntake();
                            ejecting = false;
                            startedWithFullMag = false;
                            ejectState = EjectState.IDLE;
                        } else {
                            // Continue to next shot
                            ejectState = EjectState.ROTATING_TO_LOAD;
                        }
                    }
                    break;

                case IDLE:
                default:
                    // Shouldn't really be here while ejecting=true
                    ejecting = false;
                    break;
            }
        }

        // --- Keep encoder & abs encoder aligned only when not auto-rotating ---
        if (!ejecting && autoIntakeState == AutoIntakeState.IDLE) {
            verifyAndCorrectFromAbs();
        }


        // --- Run PIDF angle control last ---
        updateAnglePID();

        return isFull();
    }

    // ========================
    // ===== PUBLIC HELP =====
    // ========================

    public boolean hasThreeBalls() {
        for (Ball b : slots) {
            if (b == Ball.EMPTY) return false;
        }
        return true;
    }

    public String getGamePattern() {
        Ball[] pattern = getPatternForTag(gameTag);
        if (pattern == null) {
            return "Fast (no pattern, tag=" + gameTag + ")";
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length; i++) {
            if (i > 0) sb.append("-");
            if (pattern[i] == Ball.PURPLE) sb.append("P");
            else if (pattern[i] == Ball.GREEN) sb.append("G");
            else sb.append("?");
        }
        return sb.toString();
    }

    public void rezeroHere() {
        zeroTicks = motor.getCurrentPosition();
        intakeIndex = 0;
        angleIntegral = 0.0;
        lastAngleErrorDeg = 0.0;
        targetAngleDeg = slotCenterAngleAtIntake(intakeIndex);
    }

    public boolean isAtMid() {
        double desiredAngle = slotCenterAngleAtLoad(ejectStartIndex);
        double currentAngle = getCurrentAngleDeg();
        double diff = smallestAngleDiff(currentAngle, normalizeAngle(desiredAngle));
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;
        return Math.abs(diff) < tolDeg;
    }

    public Ball[] getSlots() {
        return slots;
    }

    public int getEncoder() {
        return motor.getCurrentPosition();
    }

    public int getTarget() {
        return angleToTicks(targetAngleDeg);
    }

    public int getIntakeIndex() {
        return intakeIndex;
    }

    public boolean isFull() {
        for (Ball b : slots) {
            if (b == Ball.EMPTY) return false;
        }
        return true;
    }

    public boolean hasAnyBall() {
        for (Ball b : slots) {
            if (b != Ball.EMPTY) return true;
        }
        return false;
    }

    public boolean slotHasBall(int slotIndex) {
        return slots[slotIndex] != Ball.EMPTY;
    }

    public void clearSlot(int slotIndex) {
        slots[slotIndex] = Ball.EMPTY;
    }

    public int getIntakeSlotIndex() {
        return intakeIndex;
    }

    public boolean isAutoRotating() {
        return autoIntakeState != AutoIntakeState.IDLE;
    }


    public boolean isEjecting() {
        return ejecting;
    }

    public void homeToIntake() {
        commandSlotToIntake(0);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }


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

    // --- Force-mark helpers for driver debugging ---

    public void forceIntakeSlotGreen(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.GREEN;

            telemetry.addData("ForceIntake", "Slot %d forced to GREEN", intakeIndex);

            autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;

        } else {
            telemetry.addData("ForceIntake",
                    "Slot %d not empty (=%s), ignore force GREEN",
                    intakeIndex, slots[intakeIndex]);
        }
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.PURPLE;

            telemetry.addData("ForceIntake", "Slot %d forced to PURPLE", intakeIndex);

            autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;

        } else {
            telemetry.addData("ForceIntake",
                    "Slot %d not empty (=%s), ignore force PURPLE",
                    intakeIndex, slots[intakeIndex]);
        }
    }
}

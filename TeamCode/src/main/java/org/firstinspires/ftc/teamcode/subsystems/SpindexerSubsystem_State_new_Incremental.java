package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_new;


/**
 * Incremental-only Spindexer (NO ABS encoder).
 *
 * Key behavior you asked for:
 * - NO STOP_AND_RESET_ENCODER in constructor (keeps encoder continuity across OpModes).
 * - On FIRST run after RC app launch (no stored state), it DEFAULTS to "slot0 is at LOAD now",
 *   i.e. current encoder position corresponds to internal angle = 180°.
 * - State is stored in static memory so Auto -> TeleOp inherits.
 */
@com.bylazar.configurables.annotations.Configurable
public class SpindexerSubsystem_State_new_Incremental {

    // ==== Panels tunables (no external config class dependency) ====
    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Optional: let Panels override the tag; 0/21/22/23 only (else ignored)
    public static int patternTagOverride = 0;

    // ==== BALL TYPES ====
    public enum Ball { EMPTY, GREEN, PURPLE, UNKNOWN }

    private enum RawColor { RED, GREEN, PURPLE }

    private enum EjectState {
        IDLE,
        ROTATING_TO_LOAD,
        WAIT_BEFORE_LOADER,
        WAIT_AFTER_LOADER
    }

    private enum AutoIntakeState {
        IDLE,
        WAIT_FOR_ROTATE,
        ROTATING
    }

    // ==== MOTOR / GEOMETRY CONSTANTS ====
    private static final double TICKS_PER_REV = 384.5;   // goBILDA 435 rpm YJ integrated encoder
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;      // internal definition: slot0@intake = 0°
    private static final double LOAD_ANGLE   = 180.0;    // load is opposite intake
    private static final int SLOT_COUNT = 3;

    private static final int TOLERANCE_TICKS = 10;

    // PID output limits
    private static final double MAX_POWER = 0.5;
    private static final double MAX_POWER_EJECT = 0.5;

    // If mag was full when started shooting
    private static final long WAIT_BEFORE_LOADER_FULL_MS    = 400;
    private static final long WAIT_BEFORE_LOADER_PARTIAL_MS = 2000;
    private static final long WAIT_AFTER_LOADER_MS          = 300;

    // Only trust intake sensors when near the commanded target angle
    private static final double SENSE_WINDOW_DEG = 30.0;

    // Debounce so 1 noisy frame doesn't register a ball
    private static final long BALL_PRESENT_DEBOUNCE_MS = 50;

    // Timeouts so auto never deadlocks
    private static final long ROTATE_TO_LOAD_TIMEOUT_MS = 2000;
    private static final long EJECT_TOTAL_TIMEOUT_MS    = 12000;
    private static final int ROTATE_MAX_RETRIES = 2;

    // ==== HARDWARE ====
    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;

    // ==== INCREMENTAL ANGLE / ENCODER STATE ====
    // encoder value for internal angle 0° (slot0 at intake)
    private int zeroTicks = 0;

    // which slot index is currently at intake (0,1,2)
    private int intakeIndex = 0;

    // Requested target angle in the internal frame (slot0@intake = 0°)
    private double targetAngleDeg = 0.0;

    // Manual PIDF for angle control (tuned via panels)
    private PIDFCoefficients anglePIDF = new PIDFCoefficients(0, 0, 0, 0);
    private double angleIntegral = 0.0;
    private double lastAngleErrorDeg = 0.0;
    private long lastAngleUpdateTimeNs = System.nanoTime();

    // ==== SLOT / BALL STATE ====
    private final Ball[] slots = new Ball[SLOT_COUNT];
    private boolean lastBallPresent = false;

    private AutoIntakeState autoIntakeState = AutoIntakeState.IDLE;
    private int autoIntakeNextSlotIndex = 0;
    private long autoRotateTimeMs = 0;

    // ==== EJECT STATE ====
    private boolean ejecting = false;
    private EjectState ejectState = EjectState.IDLE;
    private int ejectSlotIndex = -1;
    private long ejectPhaseTime = 0;
    private boolean startedWithFullMag = false;

    private long ballPresentSinceMs = 0;
    private boolean ballPresentDebounced = false;

    private long ejectSequenceStartMs = 0;
    private long ejectStateStartMs = 0;
    private int rotateRetryCount = 0;

    private Ball pendingColor = Ball.EMPTY;
    private long pendingColorSinceMs = 0;
    private static final long COLOR_STABLE_MS = 30; // try 50–120


    // ==== PATTERN / TAG ====
    // 23 = P,P,G ; 22 = P,G,P ; 21 = G,P,P ; 0 = no pattern
    private int gameTag = 0;
    private int patternStep = 0;

    public SpindexerSubsystem_State_new_Incremental(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");

        // DO NOT reset encoder here if you want Auto -> TeleOp continuity.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.EMPTY;

        // Restore persistent state if available; otherwise DEFAULT to "slot0 is at LOAD now"
        restoreFromStorageOrDefaultToLoad();

        // Safety: hold where we are at start (no surprise motion on OpMode start)
        targetAngleDeg = getCurrentAngleDeg();
        lastAngleUpdateTimeNs = System.nanoTime();
    }

    // =========================
    // ===== STORAGE (Auto -> TeleOp) =====
    // =========================
    private void restoreFromStorageOrDefaultToLoad() {
        if (SpindexerOpModeStorage.zeroTicks != null) {
            zeroTicks = SpindexerOpModeStorage.zeroTicks;
            intakeIndex = (SpindexerOpModeStorage.intakeIndex != null) ? wrapSlot(SpindexerOpModeStorage.intakeIndex) : 0;

            if (SpindexerOpModeStorage.slotsEnc != null && SpindexerOpModeStorage.slotsEnc.length == SLOT_COUNT) {
                for (int i = 0; i < SLOT_COUNT; i++) slots[i] = decodeSlot(SpindexerOpModeStorage.slotsEnc[i]);
            }
            if (SpindexerOpModeStorage.gameTag != null) gameTag = SpindexerOpModeStorage.gameTag;
            if (SpindexerOpModeStorage.patternStep != null) patternStep = SpindexerOpModeStorage.patternStep;
        } else {
            // FIRST RUN after app launch: assume slot0 is at LOAD now (±180 from intake)
            homeSlot0AtLoadHere();
        }
    }

    private void persistToStorage() {
        SpindexerOpModeStorage.zeroTicks = zeroTicks;
        SpindexerOpModeStorage.intakeIndex = intakeIndex;
        SpindexerOpModeStorage.targetAngleDeg = targetAngleDeg;

        int[] enc = new int[SLOT_COUNT];
        for (int i = 0; i < SLOT_COUNT; i++) enc[i] = encodeSlot(slots[i]);
        SpindexerOpModeStorage.slotsEnc = enc;

        SpindexerOpModeStorage.gameTag = gameTag;
        SpindexerOpModeStorage.patternStep = patternStep;
    }

    private int encodeSlot(Ball b) {
        if (b == Ball.GREEN) return 1;
        if (b == Ball.PURPLE) return 2;
        return 0;
    }

    private Ball decodeSlot(int v) {
        if (v == 1) return Ball.GREEN;
        if (v == 2) return Ball.PURPLE;
        return Ball.EMPTY;
    }

    // =========================
    // ===== HOMING / ZEROING (incremental-only) =====
    // =========================

    /** Slot0 is physically at INTAKE now (internal angle becomes 0° at current ticks). */
    public void homeSlot0AtIntakeHere() {
        int currentTicks = motor.getCurrentPosition();
        zeroTicks = currentTicks;     // internal 0° at current ticks
        intakeIndex = 0;
        resetPidState();
        targetAngleDeg = normalizeAngle(INTAKE_ANGLE);
        persistToStorage();
    }

    /**
     * Slot0 is physically at LOAD now (your “normal offset ±180”).
     * This sets zeroTicks so that current ticks correspond to internal angle = 180°.
     */
    public void homeSlot0AtLoadHere() {
        int currentTicks = motor.getCurrentPosition();
        int internalTicks = (int) Math.round((LOAD_ANGLE / 360.0) * TICKS_PER_REV);
        zeroTicks = currentTicks - internalTicks;
        intakeIndex = 0;
        resetPidState();
        targetAngleDeg = normalizeAngle(LOAD_ANGLE);
        persistToStorage();
    }

    private void resetPidState() {
        angleIntegral = 0.0;
        lastAngleErrorDeg = 0.0;
        lastAngleUpdateTimeNs = System.nanoTime();
    }

    // =========================
    // ===== PIDF INTERFACE =====
    // =========================
    public void setPIDF(PIDFCoefficients pidf) {
        if (pidf != null) this.anglePIDF = pidf;
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.anglePIDF = new PIDFCoefficients(p, i, d, f);
    }

    // =========================
    // ===== ANGLE MATH =====
    // =========================
    private double normalizeAngle(double angleDeg) {
        return (angleDeg % 360.0 + 360.0) % 360.0;
    }

    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    private double ticksToAngle(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        return normalizeAngle(revs * 360.0);
    }

    private int angleToTicks(double angleDeg) {
        double norm = normalizeAngle(angleDeg);
        double revs = norm / 360.0;
        return zeroTicks + (int) Math.round(revs * TICKS_PER_REV);
    }

    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private double slotCenterAngleAtLoad(int slotIndex) {
        return slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
    }

    public void setTargetAngleDeg(double angleDeg) {
        this.targetAngleDeg = normalizeAngle(angleDeg);
    }

    private int wrapSlot(int idx) {
        idx %= SLOT_COUNT;
        if (idx < 0) idx += SLOT_COUNT;
        return idx;
    }

    public void commandSlotToIntake(int slotIndex) {
        slotIndex = wrapSlot(slotIndex);
        setTargetAngleDeg(slotCenterAngleAtIntake(slotIndex));
        intakeIndex = slotIndex;
    }

    public void commandSlotToLoad(int slotIndex) {
        slotIndex = wrapSlot(slotIndex);
        setTargetAngleDeg(slotCenterAngleAtLoad(slotIndex));
    }

    private boolean isAngleNear(double currentDeg, double targetDeg) {
        double diff = smallestAngleDiff(targetDeg, currentDeg);
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;
        return Math.abs(diff) <= tolDeg;
    }

    private boolean isSlotAtIntakePosition(int slotIndex) {
        return isAngleNear(getCurrentAngleDeg(), slotCenterAngleAtIntake(slotIndex));
    }

    private boolean isSlotAtLoadPosition(int slotIndex) {
        return isAngleNear(getCurrentAngleDeg(), slotCenterAngleAtLoad(slotIndex));
    }

    // =========================
    // ===== PIDF LOOP =====
    // =========================
    private void updateAnglePID() {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastAngleUpdateTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastAngleUpdateTimeNs = nowNs;

        double currentAngle = getCurrentAngleDeg();
        double errorDeg = smallestAngleDiff(targetAngleDeg, currentAngle);

        angleIntegral += errorDeg * dt;
        angleIntegral = Range.clip(angleIntegral, -100.0, 100.0);

        double derivative = (errorDeg - lastAngleErrorDeg) / dt;
        lastAngleErrorDeg = errorDeg;

        double p = anglePIDF.p, i = anglePIDF.i, d = anglePIDF.d, f = anglePIDF.f;
        double ff = f * Math.signum(errorDeg);

        double output = p * errorDeg + i * angleIntegral + d * derivative + ff;
        double maxPwr = ejecting ? MAX_POWER_EJECT : MAX_POWER;
        double power = Range.clip(output, -maxPwr, maxPwr);

        motor.setPower(power);
    }

    // =========================
    // ===== SENSING =====
    // =========================
    private boolean inSenseWindow() {
        double diff = smallestAngleDiff(getCurrentAngleDeg(), targetAngleDeg);
        return Math.abs(diff) <= SENSE_WINDOW_DEG;
    }

    private boolean rawBallPresentDistance() {
        final double THRESH_CM = 3.0;

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        return present1 || present2;
    }

    private boolean debouncedBallPresent(boolean rawPresent, long nowMs) {
        if (!rawPresent) {
            ballPresentSinceMs = 0;
            ballPresentDebounced = false;
            return false;
        }

        if (!ballPresentDebounced) {
            if (ballPresentSinceMs == 0) ballPresentSinceMs = nowMs;
            if (nowMs - ballPresentSinceMs >= BALL_PRESENT_DEBOUNCE_MS) {
                ballPresentDebounced = true;
            }
        }
        return ballPresentDebounced;
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
        if (!inSenseWindow()) return Ball.EMPTY;

        final double THRESH_CM = 5.0;
        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        if (!present1 && !present2) return Ball.EMPTY;

        RevColorSensorV3 chosen = (present1 && present2)
                ? ((d1 <= d2) ? intakeColor : intakeColor2)
                : (present1 ? intakeColor : intakeColor2);

        RawColor raw = classifyColor(chosen);
        if (raw == RawColor.GREEN) return Ball.GREEN;
        if (raw == RawColor.PURPLE) return Ball.PURPLE;
        return Ball.EMPTY; // RED = arm/junk
    }

    // =========================
    // ===== PATTERN / TAG =====
    // =========================
    public void setGameTag(int tag) { this.gameTag = tag; }
    public int getGameTag() { return gameTag; }

    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];
        switch (tag) {
            case 23: seq[0] = Ball.PURPLE; seq[1] = Ball.PURPLE; seq[2] = Ball.GREEN;  return seq;
            case 22: seq[0] = Ball.PURPLE; seq[1] = Ball.GREEN;  seq[2] = Ball.PURPLE; return seq;
            case 21: seq[0] = Ball.GREEN;  seq[1] = Ball.PURPLE; seq[2] = Ball.PURPLE; return seq;
            default: return null;
        }
    }

    // =========================
    // ===== Compatibility helpers (your TeleOp expects these) =====
    // =========================
    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean isAutoRotating() {
        return autoIntakeState != AutoIntakeState.IDLE;
    }

    public int getIntakeSlotIndex() {
        return intakeIndex;
    }

    public String getGamePattern() {
        Ball[] pattern = getPatternForTag(gameTag);
        if (pattern == null) return "Fast (no pattern, tag=" + gameTag + ")";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length; i++) {
            if (i > 0) sb.append("-");
            sb.append(pattern[i] == Ball.PURPLE ? "P" : (pattern[i] == Ball.GREEN ? "G" : "?"));
        }
        return sb.toString();
    }

    public void forceIntakeSlotGreen(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.GREEN;
            if (telemetry != null) telemetry.addData("ForceIntake", "Slot %d forced GREEN", intakeIndex);

            autoIntakeNextSlotIndex = wrapSlot(intakeIndex + 1);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
        } else {
            if (telemetry != null) telemetry.addData("ForceIntake", "Slot %d not empty", intakeIndex);
        }
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.PURPLE;
            if (telemetry != null) telemetry.addData("ForceIntake", "Slot %d forced PURPLE", intakeIndex);

            autoIntakeNextSlotIndex = wrapSlot(intakeIndex + 1);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
        } else {
            if (telemetry != null) telemetry.addData("ForceIntake", "Slot %d not empty", intakeIndex);
        }
    }

    /** TeleOp may still call this; incremental version has NO ABS, so print something harmless. */
    public void debugAbsAngle(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addData("Spd/ABS", "disabled (incremental-only)");
        telemetry.addData("Spd/zeroTicks", zeroTicks);
    }

    public void presetSlots(Ball s0, Ball s1, Ball s2) {
        slots[0] = (s0 == null) ? Ball.EMPTY : s0;
        slots[1] = (s1 == null) ? Ball.EMPTY : s1;
        slots[2] = (s2 == null) ? Ball.EMPTY : s2;
    }


    // =========================
    // ===== EJECT SELECTION =====
    // =========================
    private int selectFastestNonEmptySlot(double currentAngle) {
        int bestSlot = -1;
        double bestDiff = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (slots[i] == Ball.EMPTY) continue;

            double target = slotCenterAngleAtLoad(i);
            double diff = Math.abs(smallestAngleDiff(target, currentAngle));
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

                double target = slotCenterAngleAtLoad(i);
                double diff = Math.abs(smallestAngleDiff(target, currentAngle));
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
    // ===== MAIN UPDATE =====
    // =========================
    /**
     * Call every loop.
     * @return true if mag full after update.
     */
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverrideFromCode) {

        long now = System.currentTimeMillis();

        // Pull PIDF from panels fields
        setPIDF(
                SpindexerTuningConfig_new.kP,
                SpindexerTuningConfig_new.kI,
                SpindexerTuningConfig_new.kD,
                SpindexerTuningConfig_new.kF
        );


        // Tag override priority: Panels override if valid, else code argument if valid
        int effectiveTag = patternTagOverrideFromCode;
        if (patternTagOverride == 0 || patternTagOverride == 21 || patternTagOverride == 22 || patternTagOverride == 23) {
            effectiveTag = patternTagOverride;
        }
        if (effectiveTag == 0 || effectiveTag == 21 || effectiveTag == 22 || effectiveTag == 23) {
            gameTag = effectiveTag;
        }

        // Start eject on rising edge
        if (yEdge && !ejecting) {
            if (hasAnyBall()) {
                ejecting = true;
                ejectState = EjectState.ROTATING_TO_LOAD;
                ejectSlotIndex = -1;
                patternStep = 0;
                startedWithFullMag = isFull();
                ejectSequenceStartMs = now;
                ejectStateStartMs = now;
                rotateRetryCount = 0;
            } else {
                // fallback “pretend we have balls”
                Ball[] pattern = getPatternForTag(gameTag);
                if (pattern != null) {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = pattern[i];
                } else {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.PURPLE;
                }

                ejecting = true;
                ejectState = EjectState.ROTATING_TO_LOAD;
                ejectSlotIndex = -1;
                patternStep = 0;
                startedWithFullMag = false;
                ejectSequenceStartMs = now;
                ejectStateStartMs = now;
                rotateRetryCount = 0;
            }
        }

        // --- AUTO INTAKE ---
        boolean rawPresent = inSenseWindow() && rawBallPresentDistance();
        boolean ballPresent = debouncedBallPresent(rawPresent, now);

        switch (autoIntakeState) {
            case IDLE:
                if (!isFull()
                        && !ejecting
                        && slots[intakeIndex] == Ball.EMPTY) {

                    if (ballPresent) {
                        Ball color = detectBallColor(); // still uses your sense window + distance selection

                        if (color == Ball.GREEN || color == Ball.PURPLE) {
                            if (pendingColor != color) {
                                pendingColor = color;
                                pendingColorSinceMs = now;
                            } else if (now - pendingColorSinceMs >= COLOR_STABLE_MS) {
                                // Commit
                                slots[intakeIndex] = pendingColor;
                                if (telemetry != null) telemetry.addData("AutoIntake", "Slot %d=%s", intakeIndex, pendingColor);

                                // Schedule rotate
                                autoIntakeNextSlotIndex = wrapSlot(intakeIndex + 1);
                                autoRotateTimeMs = now + 100;
                                autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;

                                // Reset pending
                                pendingColor = Ball.EMPTY;
                                pendingColorSinceMs = 0;
                            }
                        } else {
                            // Present, but color not confidently classified this frame
                            pendingColor = Ball.EMPTY;
                            pendingColorSinceMs = 0;
                        }
                    } else {
                        pendingColor = Ball.EMPTY;
                        pendingColorSinceMs = 0;
                    }
                }
                break;


            case WAIT_FOR_ROTATE:
                if (now >= autoRotateTimeMs && !ejecting && !isFull()) {
                    setTargetAngleDeg(slotCenterAngleAtIntake(autoIntakeNextSlotIndex));
                    autoIntakeState = AutoIntakeState.ROTATING;
                }
                break;

            case ROTATING:
                if (isSlotAtIntakePosition(autoIntakeNextSlotIndex)) {
                    intakeIndex = autoIntakeNextSlotIndex;
                    autoIntakeState = AutoIntakeState.IDLE;
                }
                break;
        }

        lastBallPresent = ballPresent;

        // --- EJECT STATE MACHINE ---
        if (ejecting) {
            switch (ejectState) {

                case ROTATING_TO_LOAD: {
                    if (now - ejectSequenceStartMs > EJECT_TOTAL_TIMEOUT_MS) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        startedWithFullMag = false;
                        ejectSlotIndex = -1;
                        break;
                    }

                    if (!hasAnyBall()) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        startedWithFullMag = false;
                        ejectSlotIndex = -1;
                        break;
                    }

                    if (ejectSlotIndex < 0) {
                        ejectSlotIndex = selectNextEjectSlotIndex();
                        ejectStateStartMs = now;
                    }

                    if (ejectSlotIndex < 0 || ejectSlotIndex >= SLOT_COUNT) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        startedWithFullMag = false;
                        ejectSlotIndex = -1;
                        break;
                    }

                    commandSlotToLoad(ejectSlotIndex);

                    if (now - ejectStateStartMs > ROTATE_TO_LOAD_TIMEOUT_MS) {
                        rotateRetryCount++;
                        ejectSlotIndex = -1;
                        ejectStateStartMs = now;

                        if (rotateRetryCount > ROTATE_MAX_RETRIES) {
                            homeToIntake();
                            ejecting = false;
                            ejectState = EjectState.IDLE;
                            startedWithFullMag = false;
                            ejectSlotIndex = -1;
                            break;
                        }
                    }

                    if (isSlotAtLoadPosition(ejectSlotIndex)) {
                        long delayMs = startedWithFullMag
                                ? WAIT_BEFORE_LOADER_FULL_MS
                                : WAIT_BEFORE_LOADER_PARTIAL_MS;

                        ejectPhaseTime = now + delayMs;
                        ejectState = EjectState.WAIT_BEFORE_LOADER;

                        startedWithFullMag = true;
                        rotateRetryCount = 0;
                    }
                    break;
                }

                case WAIT_BEFORE_LOADER:
                    if (now >= ejectPhaseTime) {
                        if (loader != null) loader.startCycle();
                        ejectPhaseTime = now + WAIT_AFTER_LOADER_MS;
                        ejectState = EjectState.WAIT_AFTER_LOADER;
                    }
                    break;

                case WAIT_AFTER_LOADER:
                    if (now >= ejectPhaseTime) {
                        // clear shot slot
                        if (ejectSlotIndex >= 0 && ejectSlotIndex < SLOT_COUNT) {
                            slots[ejectSlotIndex] = Ball.EMPTY;
                        }

                        Ball[] pattern = getPatternForTag(gameTag);
                        if (pattern != null) patternStep++;

                        boolean usingPattern = (pattern != null);
                        boolean done = !hasAnyBall() || (usingPattern && patternStep >= pattern.length);

                        if (done) {
                            homeToIntake();
                            ejecting = false;
                            startedWithFullMag = false;
                            ejectState = EjectState.IDLE;
                            ejectSlotIndex = -1;
                        } else {
                            ejectState = EjectState.ROTATING_TO_LOAD;
                            ejectSlotIndex = -1;
                            ejectStateStartMs = now;
                        }
                    }
                    break;

                case IDLE:
                default:
                    ejecting = false;
                    break;
            }
        }

        // Run PID last
        updateAnglePID();

        // persist for next OpMode
        persistToStorage();

        return isFull();
    }

    // =========================
    // ===== Public helpers =====
    // =========================
    public void homeToIntake() {
        commandSlotToIntake(0);
    }

    public boolean isEjecting() { return ejecting; }

    public Ball[] getSlots() { return slots; }

    public int getEncoder() { return motor.getCurrentPosition(); }

    public int getTarget() { return angleToTicks(targetAngleDeg); }

    public int getIntakeIndex() { return intakeIndex; }

    public double getTargetAngleDeg() { return targetAngleDeg; }

    // =========================
    // ===== Static storage class (persists across OpModes in same RC app session) =====
    // =========================
    static class SpindexerOpModeStorage {
        static Integer zeroTicks = null;
        static Integer intakeIndex = null;
        static Double targetAngleDeg = null;
        static int[] slotsEnc = null;

        static Integer gameTag = null;
        static Integer patternStep = null;
    }

    public void debugColorTelemetry(Telemetry telemetry, long nowMs) {
        if (telemetry == null) return;

        // --- Angles / sense window ---
        double curAng = getCurrentAngleDeg();
        double tgtAng = getTargetAngleDeg();
        double diff = smallestAngleDiff(curAng, tgtAng);
        boolean sense = inSenseWindow();

        telemetry.addData("Spd/Sense", "win=%s cur=%.1f tgt=%.1f diff=%.1f",
                sense, curAng, tgtAng, diff);

        // --- Distances / presence ---
        final double THRESH_CM = 5.0; // matches your code; change for testing
        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        telemetry.addData("Spd/Dist", "d1=%.1f(%s) d2=%.1f(%s) thr=%.1fcm",
                d1, present1 ? "IN" : "OUT",
                d2, present2 ? "IN" : "OUT",
                THRESH_CM);

        // --- Debounce state ---
        boolean rawPresent = sense && (present1 || present2);
        boolean debounced = debouncedBallPresent(rawPresent, nowMs);
        telemetry.addData("Spd/Present", "raw=%s debounced=%s last=%s",
                rawPresent, debounced, lastBallPresent);

        // --- Per-sensor RGB + classification ---
        // Sensor 1
        int r1 = intakeColor.red();
        int g1 = intakeColor.green();
        int b1 = intakeColor.blue();
        int sum1 = r1 + g1 + b1;

        double rn1 = (sum1 > 0) ? (r1 / (double) sum1) : 0.0;
        double gn1 = (sum1 > 0) ? (g1 / (double) sum1) : 0.0;
        double bn1 = (sum1 > 0) ? (b1 / (double) sum1) : 0.0;

        RawColor raw1 = classifyColor(intakeColor);

        telemetry.addData("Spd/RGB1", "r=%d g=%d b=%d sum=%d | rn=%.2f gn=%.2f bn=%.2f | %s",
                r1, g1, b1, sum1, rn1, gn1, bn1, raw1);

        // Sensor 2
        int r2 = intakeColor2.red();
        int g2 = intakeColor2.green();
        int b2 = intakeColor2.blue();
        int sum2 = r2 + g2 + b2;

        double rn2 = (sum2 > 0) ? (r2 / (double) sum2) : 0.0;
        double gn2 = (sum2 > 0) ? (g2 / (double) sum2) : 0.0;
        double bn2 = (sum2 > 0) ? (b2 / (double) sum2) : 0.0;

        RawColor raw2 = classifyColor(intakeColor2);

        telemetry.addData("Spd/RGB2", "r=%d g=%d b=%d sum=%d | rn=%.2f gn=%.2f bn=%.2f | %s",
                r2, g2, b2, sum2, rn2, gn2, bn2, raw2);

        // --- Which sensor chosen + final detected ball ---
        RevColorSensorV3 chosen = null;
        String chosenName = "NONE";

        if (sense && (present1 || present2)) {
            chosen = (present1 && present2)
                    ? ((d1 <= d2) ? intakeColor : intakeColor2)
                    : (present1 ? intakeColor : intakeColor2);
            chosenName = (chosen == intakeColor) ? "S1" : "S2";
        }

        Ball detected = detectBallColor();
        telemetry.addData("Spd/Chosen", "chosen=%s detected=%s", chosenName, detected);
    }

}

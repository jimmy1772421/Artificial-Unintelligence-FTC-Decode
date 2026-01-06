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

public class SpindexerSubsystem_State_new_AbsOnly {

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

    // ==== GEOMETRY ====
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;   // slot 0 center at intake = 0 deg
    private static final double LOAD_ANGLE   = 180.0; // load is +180 from intake reference
    private static final int SLOT_COUNT = 3;

    // ==== ABS ====
    private static final double ABS_VREF = 3.3; // analog reference
    // Raw abs angle when SLOT0 is perfectly at intake (internal 0 deg)
    private static final double ABS_MECH_OFFSET_DEG = (144.7+180); // calibrate

    // ==== CONTROL ====
    private static final double MAX_POWER = 0.5;
    private static final double MAX_POWER_EJECT = 0.7;

    // "Near target" tolerance in degrees
    // (your old ticks tolerance ~ 3-4 deg; adjust as needed)
    private static final double ANGLE_TOL_DEG = 4.0;

    // Debounce so 1 noisy frame doesn't register a ball
    private static final long BALL_PRESENT_DEBOUNCE_MS = 80;
    // Only trust intake sensors when near target
    private static final double SENSE_WINDOW_DEG = 30.0;

    // Eject timing
    private static final long WAIT_BEFORE_LOADER_FULL_MS    = 700;
    private static final long WAIT_BEFORE_LOADER_PARTIAL_MS = 2000;
    private static final long WAIT_AFTER_LOADER_MS          = 400;

    // Global safety
    private static final long EJECT_TOTAL_TIMEOUT_MS = 8000;

    // Rotation timeout behavior
    public static long ROTATE_TO_LOAD_TIMEOUT_MS = 1200;
    public static double LOAD_ACCEPT_TOL_DEG_ON_TIMEOUT = 8.0;
    private static final int ROTATE_MAX_RETRIES = 2;

    // ==== HARDWARE ====
    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    // ==== STATE ====
    private final Ball[] slots = new Ball[SLOT_COUNT];

    private int intakeIndex = 0;            // which slot is at intake
    private double targetAngleDeg = 0.0;    // internal absolute angle target [0..360)

    private PIDFCoefficients anglePIDF = new PIDFCoefficients(0, 0, 0, 0);
    private double angleIntegral = 0.0;
    private double lastAngleErrorDeg = 0.0;
    private long lastAngleUpdateTimeNs = System.nanoTime();

    private boolean lastBallPresent = false;
    private AutoIntakeState autoIntakeState = AutoIntakeState.IDLE;
    private int autoIntakeNextSlotIndex = 0;
    private long autoRotateTimeMs = 0;

    private boolean ejecting = false;
    private EjectState ejectState = EjectState.IDLE;
    private int ejectSlotIndex = -1;
    private long ejectPhaseTime = 0;
    private boolean startedWithFullMag = false;

    private int patternStep = 0;
    private long ejectStateStartMs = 0;
    private long ejectSequenceStartMs = 0;
    private int rotateRetryCount = 0;

    // 23=P,P,G  22=P,G,P  21=G,P,P  0=no pattern
    private int gameTag = 0;

    private long ballPresentSinceMs = 0;
    private boolean ballPresentDebounced = false;

    public SpindexerSubsystem_State_new_AbsOnly(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");
        absEncoder = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);

        for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.EMPTY;

        // Initialize intakeIndex from ABS immediately (ABS-only design)
        double a = getCurrentAngleDeg();
        intakeIndex = nearestIntakeSlotIndexFromAngle(a);

        // Start holding current slot at intake
        targetAngleDeg = slotCenterAngleAtIntake(intakeIndex);

        lastAngleUpdateTimeNs = System.nanoTime();
    }

    // ==========================
    // ===== PIDF INTERFACE =====
    // ==========================
    public void setPIDF(PIDFCoefficients pidf) {
        if (pidf != null) this.anglePIDF = pidf;
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.anglePIDF = new PIDFCoefficients(p, i, d, f);
    }

    public PIDFCoefficients getPIDF() { return anglePIDF; }

    // ==========================
    // ===== ABS ANGLE MATH =====
    // ==========================
    private double readAbsAngleDeg_0to360() {
        double v = absEncoder.getVoltage();
        double angle = (v / ABS_VREF) * 360.0;
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    /** normalize any angle to [0, 360) */
    private double normalizeAngle(double angleDeg) {
        return (angleDeg % 360.0 + 360.0) % 360.0;
    }

    /** smallest signed diff a-b in [-180, 180) */
    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    /**
     * Internal ABS angle in degrees [0..360):
     * 0° = slot0 center at intake.
     */
    public double getCurrentAngleDeg() {
        double raw = readAbsAngleDeg_0to360();
        return normalizeAngle(raw - ABS_MECH_OFFSET_DEG);
    }

    private int nearestIntakeSlotIndexFromAngle(double internalAngleDeg) {
        // nearest among 0,120,240
        int nearest = (int) Math.round(internalAngleDeg / DEGREES_PER_SLOT) % SLOT_COUNT;
        if (nearest < 0) nearest += SLOT_COUNT;
        return nearest;
    }

    // =========================
    // ===== TARGET HELPERS ====
    // =========================
    private double slotCenterAngleAtIntake(int slotIndex) {
        return normalizeAngle(INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT);
    }

    private double slotCenterAngleAtLoad(int slotIndex) {
        return normalizeAngle(slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE));
    }

    public void setTargetAngleDeg(double angleDeg) {
        this.targetAngleDeg = normalizeAngle(angleDeg);
    }

    public void commandSlotToIntake(int slotIndex) {
        slotIndex = ((slotIndex % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        setTargetAngleDeg(slotCenterAngleAtIntake(slotIndex));
        intakeIndex = slotIndex;
    }

    public void commandSlotToLoad(int slotIndex) {
        slotIndex = ((slotIndex % SLOT_COUNT) + SLOT_COUNT) % SLOT_COUNT;
        setTargetAngleDeg(slotCenterAngleAtLoad(slotIndex));
    }

    private boolean isAngleNear(double currentDeg, double targetDeg) {
        double diff = smallestAngleDiff(targetDeg, currentDeg);
        return Math.abs(diff) <= ANGLE_TOL_DEG;
    }

    private boolean isSlotAtIntakePosition(int slotIndex) {
        return isAngleNear(getCurrentAngleDeg(), slotCenterAngleAtIntake(slotIndex));
    }

    private boolean isSlotAtLoadPosition(int slotIndex) {
        return isAngleNear(getCurrentAngleDeg(), slotCenterAngleAtLoad(slotIndex));
    }

    // =========================
    // ===== PIDF LOOP ========
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

        double p = anglePIDF.p;
        double i = anglePIDF.i;
        double d = anglePIDF.d;
        double f = anglePIDF.f;

        double ff = f * Math.signum(errorDeg);

        double output = p * errorDeg + i * angleIntegral + d * derivative + ff;
        double maxPwr = ejecting ? MAX_POWER_EJECT : MAX_POWER;
        double power = Range.clip(output, -maxPwr, maxPwr);

        motor.setPower(power);
    }

    // =========================
    // ===== COLOR / BALLS =====
    // =========================
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
        if (redDominant)   return RawColor.RED;
        return RawColor.PURPLE;
    }

    private boolean rawBallPresentDistance() {
        final double THRESH_CM = 5.0;

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
            if (nowMs - ballPresentSinceMs >= BALL_PRESENT_DEBOUNCE_MS) ballPresentDebounced = true;
        }
        return ballPresentDebounced;
    }

    private boolean inSenseWindow() {
        double diff = smallestAngleDiff(targetAngleDeg, getCurrentAngleDeg());
        return Math.abs(diff) <= SENSE_WINDOW_DEG;
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
        switch (raw) {
            case GREEN:  return Ball.GREEN;
            case PURPLE: return Ball.PURPLE;
            case RED:
            default:     return Ball.EMPTY;
        }
    }

    // ================================
    // ===== TAG / PATTERN HELPERS ====
    // ================================
    public void setGameTag(int tag) { this.gameTag = tag; }
    public int getGameTag() { return gameTag; }

    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];
        switch (tag) {
            case 23: seq[0]=Ball.PURPLE; seq[1]=Ball.PURPLE; seq[2]=Ball.GREEN;  return seq;
            case 22: seq[0]=Ball.PURPLE; seq[1]=Ball.GREEN;  seq[2]=Ball.PURPLE; return seq;
            case 21: seq[0]=Ball.GREEN;  seq[1]=Ball.PURPLE; seq[2]=Ball.PURPLE; return seq;
            default: return null;
        }
    }

    private int selectFastestNonEmptySlot(double currentAngle) {
        int bestSlot = -1;
        double bestDiff = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (!slotHasBall(i)) continue;
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

    // ================================
    // ===== MAIN UPDATE (STATE) ======
    // ================================
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        long now = System.currentTimeMillis();

        // Sync PIDF from Panels each loop (same as your other class)
        setPIDF(
                SpindexerTuningConfig_new.kP,
                SpindexerTuningConfig_new.kI,
                SpindexerTuningConfig_new.kD,
                SpindexerTuningConfig_new.kF
        );

        // Allow Panels to override tag too
        int effectiveTag = patternTagOverride;
        int cfgTag = SpindexerTuningConfig_new.patternTagOverride;
        if (cfgTag == 0 || cfgTag == 21 || cfgTag == 22 || cfgTag == 23) {
            effectiveTag = cfgTag;
        }
        if (effectiveTag == 0 || effectiveTag == 21 || effectiveTag == 22 || effectiveTag == 23) {
            gameTag = effectiveTag;
        }

        // Start eject sequence
        if (yEdge && !ejecting) {
            if (!hasAnyBall()) {
                // Fallback: pretend balls based on tag/pattern or default PURPLE
                Ball[] pattern = getPatternForTag(gameTag);
                if (pattern != null) {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = pattern[i];
                } else {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.PURPLE;
                }
                startedWithFullMag = false;
            } else {
                startedWithFullMag = isFull();
            }

            ejecting = true;
            ejectState = EjectState.ROTATING_TO_LOAD;
            patternStep = 0;
            ejectSlotIndex = -1;
            ejectStateStartMs = now;
            ejectSequenceStartMs = now;
            rotateRetryCount = 0;
        }

        // --- AUTO INTAKE ---
        boolean rawPresent = inSenseWindow() && rawBallPresentDistance();
        boolean ballPresent = debouncedBallPresent(rawPresent, now);

        switch (autoIntakeState) {
            case IDLE:
                if (!isFull()
                        && ballPresent
                        && !lastBallPresent
                        && slots[intakeIndex] == Ball.EMPTY
                        && !ejecting) {

                    Ball color = detectBallColor();
                    if (color == Ball.GREEN || color == Ball.PURPLE) {
                        slots[intakeIndex] = color;

                        autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
                        autoRotateTimeMs = now + 100;
                        autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
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
                    // global timeout
                    if (now - ejectSequenceStartMs > EJECT_TOTAL_TIMEOUT_MS) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        ejectSlotIndex = -1;
                        break;
                    }

                    if (!hasAnyBall()) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        ejectSlotIndex = -1;
                        break;
                    }

                    // latch slot
                    if (ejectSlotIndex < 0) {
                        ejectSlotIndex = selectNextEjectSlotIndex();
                        ejectStateStartMs = now;
                    }

                    if (ejectSlotIndex < 0 || ejectSlotIndex >= SLOT_COUNT) {
                        homeToIntake();
                        ejecting = false;
                        ejectState = EjectState.IDLE;
                        ejectSlotIndex = -1;
                        break;
                    }

                    // command load
                    commandSlotToLoad(ejectSlotIndex);

                    // timeout handling (ABS-only)
                    if (now - ejectStateStartMs > ROTATE_TO_LOAD_TIMEOUT_MS) {
                        double cur = getCurrentAngleDeg();
                        double tgt = slotCenterAngleAtLoad(ejectSlotIndex);
                        double err = Math.abs(smallestAngleDiff(tgt, cur));

                        if (err <= LOAD_ACCEPT_TOL_DEG_ON_TIMEOUT) {
                            long delayMs = startedWithFullMag ? WAIT_BEFORE_LOADER_FULL_MS : WAIT_BEFORE_LOADER_PARTIAL_MS;
                            ejectPhaseTime = now + delayMs;
                            ejectState = EjectState.WAIT_BEFORE_LOADER;
                            startedWithFullMag = true;
                            rotateRetryCount = 0;
                            break;
                        }

                        rotateRetryCount++;
                        ejectSlotIndex = -1;
                        ejectStateStartMs = now;

                        if (rotateRetryCount > ROTATE_MAX_RETRIES) {
                            homeToIntake();
                            ejecting = false;
                            ejectState = EjectState.IDLE;
                            ejectSlotIndex = -1;
                            break;
                        }
                    }

                    // success
                    if (isSlotAtLoadPosition(ejectSlotIndex)) {
                        long delayMs = startedWithFullMag ? WAIT_BEFORE_LOADER_FULL_MS : WAIT_BEFORE_LOADER_PARTIAL_MS;
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
                        clearSlot(ejectSlotIndex);
                        ejectSlotIndex = -1;

                        Ball[] pattern = getPatternForTag(gameTag);
                        boolean usingPattern = (pattern != null);
                        if (usingPattern) patternStep++;

                        if (!hasAnyBall() || (usingPattern && patternStep >= pattern.length)) {
                            homeToIntake();
                            ejecting = false;
                            startedWithFullMag = false;
                            ejectState = EjectState.IDLE;
                        } else {
                            ejectState = EjectState.ROTATING_TO_LOAD;
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

        // PID last
        updateAnglePID();

        return isFull();
    }

    // ========================
    // ===== PUBLIC API =======
    // ========================
    public Ball[] getSlots() { return slots; }

    // ABS-only: "encoder" is not meaningful; keep for compatibility
    public int getEncoder() { return 0; }

    // ABS-only: "target ticks" not meaningful; keep for compatibility
    public int getTarget() { return 0; }

    public int getIntakeIndex() { return intakeIndex; }
    public int getIntakeSlotIndex() { return intakeIndex; }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public boolean slotHasBall(int slotIndex) { return slots[slotIndex] != Ball.EMPTY; }
    public void clearSlot(int slotIndex) { slots[slotIndex] = Ball.EMPTY; }

    public boolean isAutoRotating() { return autoIntakeState != AutoIntakeState.IDLE; }
    public boolean isEjecting() { return ejecting; }

    public void homeToIntake() { commandSlotToIntake(0); }

    public double getTargetAngleDeg() { return targetAngleDeg; }

    public void rezeroHere() {
        // ABS-only: no zeroTicks concept. "rezero" means: treat current ABS internal as 0
        // If you want that behavior, you’d change ABS_MECH_OFFSET_DEG dynamically.
        // We intentionally do NOTHING here to keep behavior predictable.
        angleIntegral = 0.0;
        lastAngleErrorDeg = 0.0;
        targetAngleDeg = slotCenterAngleAtIntake(intakeIndex);
    }

    // Keep old helper shape
    public boolean isAtMid() {
        double desiredAngle = slotCenterAngleAtLoad(0);
        double cur = getCurrentAngleDeg();
        return Math.abs(smallestAngleDiff(desiredAngle, cur)) <= ANGLE_TOL_DEG;
    }

    // ===== DEBUG =====
    public void debugAbsAngle(Telemetry telemetry) {
        double raw = readAbsAngleDeg_0to360();
        double internal = getCurrentAngleDeg();
        telemetry.addData("Abs raw", "%.1f deg", raw);
        telemetry.addData("Abs internal(slot0@intake=0)", "%.1f deg", internal);
        telemetry.addData("Abs offset", "%.1f deg", ABS_MECH_OFFSET_DEG);
        telemetry.addData("Intake slot index", intakeIndex);
    }

    public void setCoast(boolean coast) {
        motor.setPower(0.0);
        motor.setZeroPowerBehavior(coast ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void forceIntakeSlotGreen(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.GREEN;
            autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
        }
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        if (slots[intakeIndex] == Ball.EMPTY) {
            slots[intakeIndex] = Ball.PURPLE;
            autoIntakeNextSlotIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
            autoRotateTimeMs = System.currentTimeMillis() + 100;
            autoIntakeState = AutoIntakeState.WAIT_FOR_ROTATE;
        }
    }

    public void presetSlots(Ball s0, Ball s1, Ball s2) {
        slots[0] = (s0 == null) ? Ball.EMPTY : s0;
        slots[1] = (s1 == null) ? Ball.EMPTY : s1;
        slots[2] = (s2 == null) ? Ball.EMPTY : s2;
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
}

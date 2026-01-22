package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PASSIVE Spindexer (frame-mounted sensors, trust only when aligned at intake).
 *
 * Your physical rules:
 * - Degrees increase clockwise (CW).
 * - CCW = EJECT direction.
 * - CW = SAFE direction (won't eject).
 *
 * Mechanism note:
 * - You park "past" preshoot (staging) via SAFE CW, then come back to preshoot via CCW
 *   so the rod can dangle down before the 720° CCW shoot.
 *
 * Compatibility:
 * - Keeps update(...) signature and common getters used by old TeleOps/Autos.
 * - Keeps slots[] array (0/1/2) representing POCKETS in the wheel frame (slot0 at intake when angle=0).
 */
@com.bylazar.configurables.annotations.Configurable
public class SpindexerSubsystem_Passive_State_new_Incremental {

    // =========================
    // ===== CONFIG / TUNABLES ==
    // =========================

    /**
     * One switch to fix direction:
     * - If +power makes ticks go the WRONG way for your CW-positive convention, set this true.
     * After flipping this, re-home (homeSlot0AtIntakeHere()) because angle mapping sign changes.
     */
    public static boolean REVERSE_MOTOR = true;

    // ===== OLD incremental PID defaults you gave me =====
    // (Used for both MOVE and HOLD; HOLD is limited by HOLD_MAX_POWER.)
    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.12;

    // Integral clamp (anti-windup)
    public static double I_CLAMP = 200.0;

    // ===== Panels slot simulation / override (for testing without Brushlands) =====
    public static boolean USE_PANELS_SLOT_OVERRIDE = false;
    /** 0=EMPTY, 1=GREEN, 2=PURPLE */
    public static int OVERRIDE_SLOT0 = 0;
    public static int OVERRIDE_SLOT1 = 0;
    public static int OVERRIDE_SLOT2 = 0;

    /** If true, overrides only apply when aligned at intake (recommended). */
    public static boolean OVERRIDE_ONLY_WHEN_ALIGNED = true;

    /** If true, when OVERRIDE_SLOT1 or OVERRIDE_SLOT2 changes EMPTY->BALL, we force slot0 to re-read. */
    public static boolean OVERRIDE_FORCE_SLOT0_REREAD = true;

    // Internal: last override values so we can detect changes
    private int lastOv0 = -999, lastOv1 = -999, lastOv2 = -999;

    private Ball decodeOverride(int v) {
        if (v == 1) return Ball.GREEN;
        if (v == 2) return Ball.PURPLE;
        return Ball.EMPTY;
    }

    // Motor encoder
    private static final double TICKS_PER_REV = 384.5; // goBILDA 435rpm YJ integrated encoder
    private static final double DEG_PER_TICK = 360.0 / TICKS_PER_REV;

    // Geometry (wheel-frame pockets)
    private static final int SLOT_COUNT = 3;
    private static final double DEGREES_PER_SLOT = 120.0;

    // World angles (by your convention)
    public static double INTAKE_ANGLE_DEG   = 0.0;   // slot0 at intake when wheel angle = 0
    public static double PRESHOOT_ANGLE_DEG = 200.0; // preshoot
    /** "Staging" is defined as PRESHOOT + GO_PAST (CW safe). Example: GO_PAST=60 -> 240 staging. */
    public static double GO_PAST_ANGLE_DEG  = 40.0;

    // Trust sensors only when aligned to intake within this many degrees
    public static double ALIGN_TOL_DEG = 10.0;

    // Slot0 (Rev) presence threshold
    public static double SLOT0_DIST_THRESH_CM = 3.0;

    // Frame-based sensing (presence+color together)
    public static int PRESENT_FRAMES_REQUIRED = 2;
    public static int COLOR_FRAMES_REQUIRED   = 2;

    // Movement control
    public static double MOVE_MAX_POWER = 0.55;
    public static double MOVE_DONE_TOL_DEG = 2.0;

    // If we overshoot while doing a "preferred direction" move, allow a small correction
    // instead of wrapping 350°.
    public static double OVERSHOOT_CORRECT_MAX_DEG = 60.0;

    // Hold control
    public static double HOLD_MAX_POWER = 0.30;

    // Shooting (open-loop, encoder-distance tracked)
    public static double SHOOT_DEG = 1080.0;        // 3 revs
    public static double SHOOT_POWER_HIGH = 1.0;
    public static double SHOOT_POWER_LOW  = 0.9;
    public static long   SHOOT_HIGH_MS    = 300;
    public static long   SHOOT_TIMEOUT_MS = 4500;

    // Pattern override (same idea as your old code)
    // -1 means no override; else 0/21/22/23
    public static int patternTagOverride = -1;

    // ===== DEBUG =====
    private double dbgLastRequestedPower = 0.0; // "angle-space" request before REVERSE
    private double dbgLastAppliedPower   = 0.0; // actual motor.setPower value after REVERSE+clip
    private double dbgLastErrShortestDeg = 0.0; // shortest-path error
    private double dbgLastErrUsedDeg     = 0.0; // error after "prefer CW/CCW" logic

    public double dbgLastRequestedPower() { return dbgLastRequestedPower; }
    public double dbgLastAppliedPower()   { return dbgLastAppliedPower; }
    public double dbgLastErrShortestDeg() { return dbgLastErrShortestDeg; }
    public double dbgLastErrUsedDeg()     { return dbgLastErrUsedDeg; }

    // =========================
    // ===== HARDWARE NAMES =====
    // =========================
    public static String SLOT1_PRESENT_NAME = "slot1Present";
    public static String SLOT1_GREEN_NAME   = "slot1Green";
    public static String SLOT2_PRESENT_NAME = "slot2Present";
    public static String SLOT2_GREEN_NAME   = "slot2Green";

    // =========================
    // ===== SLOT MODEL =========
    // =========================
    public enum Ball { EMPTY, GREEN, PURPLE, UNKNOWN }

    private enum RawColor { RED, GREEN, PURPLE }

    // Provided by you (CCW eject order):
    // start slot0 => 0,2,1
    // start slot1 => 1,0,2
    // start slot2 => 2,1,0
    private static final int[][] EJECT_ORDER = new int[][]{
            {0, 2, 1},
            {1, 0, 2},
            {2, 1, 0}
    };

    // =========================
    // ===== STATE MACHINE ======
    // =========================
    private enum State {
        IDLE,
        POSITION_TO_PARK_CW,
        POSITION_TO_PRESHOOT_CCW,
        READY,
        SHOOTING,
        RETURNING_CW
    }

    private State state = State.IDLE;
    private State lastState = null;

    // =========================
    // ===== HARDWARE ===========
    // =========================
    private final DcMotorEx motor;
    private final RevColorSensorV3 slot0Color1;
    private final RevColorSensorV3 slot0Color2;

    private final DigitalChannel slot1Present;
    private final DigitalChannel slot1Green;
    private final DigitalChannel slot2Present;
    private final DigitalChannel slot2Green;

    // =========================
    // ===== ANGLE / ZERO =======
    // =========================
    private int zeroTicks = 0;          // raw motor ticks corresponding to wheel angle = 0 (slot0 at intake)
    private double targetAngleDeg = 0.0;

    // Persist across opmodes in same RC session
    static class SpindexerOpModeStorage {
        static Integer zeroTicks = null;
        static Double  targetAngleDeg = null;
        static Integer gameTag = null;
        static int[]   slotsEnc = null;
        static Boolean reverseMotor = null; // detect flipping between runs
    }

    // =========================
    // ===== SLOT STATE =========
    // =========================
    private final Ball[] slots = new Ball[SLOT_COUNT];

    // Frame-based counters for slot0
    private int slot0PresentFrames = 0;
    private int slot0ColorFrames = 0;
    private Ball slot0LastColorFrame = Ball.EMPTY;

    // Frame-based counters for slot1/2
    private final int[] presentFrames = new int[SLOT_COUNT];
    private final int[] colorFrames = new int[SLOT_COUNT];
    private final Ball[] lastColorFrame = new Ball[SLOT_COUNT];

    // Reread rule for slot0 when slot1/2 gets newly filled
    private boolean slot0NeedsReread = false;

    // Tag/pattern
    private int gameTag = 0;

    // Position plan
    private int selectedStartSlot = 0;
    private double parkWheelAngleDeg = 0.0;
    private double preshootWheelAngleDeg = 0.0;

    // Shoot request latch (if button pressed early)
    private boolean shootRequested = false;

    // Shooting tracking
    private boolean ejecting = false;
    private long shootStartMs = 0;
    private int shootLastTicks = 0;
    private double shootMovedDeg = 0.0;

    // =========================
    // ===== PID STATE ==========
    // =========================
    private double pidIntegral = 0.0;
    private double pidLastErrDeg = 0.0;
    private long pidLastNs = System.nanoTime();

    private void resetDrivePID() {
        pidIntegral = 0.0;
        pidLastErrDeg = 0.0;
        pidLastNs = System.nanoTime();
    }

    private enum ErrorMode {
        SHORTEST,
        PREFER_CW,
        PREFER_CCW
    }

    // =========================
    // ===== CONSTRUCTOR ========
    // =========================
    public SpindexerSubsystem_Passive_State_new_Incremental(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slot0Color1 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        slot0Color2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");

        slot1Present = hardwareMap.get(DigitalChannel.class, SLOT1_PRESENT_NAME);
        slot1Green   = hardwareMap.get(DigitalChannel.class, SLOT1_GREEN_NAME);
        slot2Present = hardwareMap.get(DigitalChannel.class, SLOT2_PRESENT_NAME);
        slot2Green   = hardwareMap.get(DigitalChannel.class, SLOT2_GREEN_NAME);

        slot1Present.setMode(DigitalChannel.Mode.INPUT);
        slot1Green.setMode(DigitalChannel.Mode.INPUT);
        slot2Present.setMode(DigitalChannel.Mode.INPUT);
        slot2Green.setMode(DigitalChannel.Mode.INPUT);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
            presentFrames[i] = 0;
            colorFrames[i] = 0;
            lastColorFrame[i] = Ball.EMPTY;
        }

        restoreFromStorageOrDefaultToIntake();

        // Hold where we are on boot
        targetAngleDeg = getCurrentAngleDeg();
        state = State.IDLE;
        ejecting = false;
        lastState = state;
        resetDrivePID();
    }

    private void restoreFromStorageOrDefaultToIntake() {
        if (SpindexerOpModeStorage.zeroTicks != null) {
            zeroTicks = SpindexerOpModeStorage.zeroTicks;
            if (SpindexerOpModeStorage.targetAngleDeg != null) targetAngleDeg = SpindexerOpModeStorage.targetAngleDeg;
            if (SpindexerOpModeStorage.gameTag != null) gameTag = SpindexerOpModeStorage.gameTag;

            if (SpindexerOpModeStorage.slotsEnc != null && SpindexerOpModeStorage.slotsEnc.length == SLOT_COUNT) {
                for (int i = 0; i < SLOT_COUNT; i++) slots[i] = decodeSlot(SpindexerOpModeStorage.slotsEnc[i]);
            }
        } else {
            // First run: assume current wheel position is INTAKE (slot0 at intake now)
            homeSlot0AtIntakeHere();
        }

        // If user flips REVERSE_MOTOR between runs, stored zero becomes ambiguous.
        if (SpindexerOpModeStorage.reverseMotor != null && SpindexerOpModeStorage.reverseMotor != REVERSE_MOTOR) {
            homeSlot0AtIntakeHere();
        }
    }

    private void persistToStorage() {
        SpindexerOpModeStorage.zeroTicks = zeroTicks;
        SpindexerOpModeStorage.targetAngleDeg = targetAngleDeg;
        SpindexerOpModeStorage.gameTag = gameTag;
        SpindexerOpModeStorage.reverseMotor = REVERSE_MOTOR;

        int[] enc = new int[SLOT_COUNT];
        for (int i = 0; i < SLOT_COUNT; i++) enc[i] = encodeSlot(slots[i]);
        SpindexerOpModeStorage.slotsEnc = enc;
    }

    private int encodeSlot(Ball b) {
        if (b == Ball.GREEN) return 1;
        if (b == Ball.PURPLE) return 2;
        if (b == Ball.UNKNOWN) return 3;
        return 0;
    }

    private Ball decodeSlot(int v) {
        if (v == 1) return Ball.GREEN;
        if (v == 2) return Ball.PURPLE;
        if (v == 3) return Ball.UNKNOWN;
        return Ball.EMPTY;
    }

    // =========================
    // ===== PUBLIC API (compat)
    // =========================
    public Ball[] getSlots() { return slots; }

    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean isEjecting() { return ejecting; }

    public int getGameTag() { return gameTag; }

    public void setGameTag(int tag) { this.gameTag = tag; }

    public String getGamePattern() {
        Ball[] p = getPatternForTag(gameTag);
        if (p == null) return "Fast (no pattern, tag=" + gameTag + ")";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < p.length; i++) {
            if (i > 0) sb.append("-");
            sb.append(p[i] == Ball.PURPLE ? "P" : (p[i] == Ball.GREEN ? "G" : "?"));
        }
        return sb.toString();
    }

    /** Same name as before; now it simply commands RETURN to intake (0°). */
    public void homeToIntake() {
        shootRequested = false;
        ejecting = false;
        state = State.RETURNING_CW;
        targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);
        // PID reset on new state will happen in update()
    }

    /** Hard re-zero: declare current position is intake (slot0 at intake). */
    public void homeSlot0AtIntakeHere() {
        zeroTicks = motor.getCurrentPosition();
        targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);
        state = State.IDLE;

        clearAllSensingCounters();
        slot0NeedsReread = true;

        resetDrivePID();
        persistToStorage();
    }

    /** Optional compat: "Load" in old code was 180; here treat as preshoot hold. */
    public void homeSlot0AtLoadHere() {
        targetAngleDeg = normalizeAngle(PRESHOOT_ANGLE_DEG);
        state = State.IDLE;
        resetDrivePID();
        persistToStorage();
    }

    public int getEncoder() { return motor.getCurrentPosition(); }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public int getTarget() { return angleToTicks(targetAngleDeg); }

    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
    }

    /** Signed smallest-path error (target - current) in degrees in [-180,180]. */
    public double getAngleErrorToTargetDeg() {
        return smallestAngleDiff(targetAngleDeg, getCurrentAngleDeg());
    }

    public boolean isAutoRotating() {
        return state == State.POSITION_TO_PARK_CW
                || state == State.POSITION_TO_PRESHOOT_CCW
                || state == State.RETURNING_CW;
    }

    public int getIntakeIndex() { return 0; } // pocket index at intake when aligned (wheel frame)

    public void presetSlots(Ball s0, Ball s1, Ball s2) {
        slots[0] = (s0 == null) ? Ball.EMPTY : s0;
        slots[1] = (s1 == null) ? Ball.EMPTY : s1;
        slots[2] = (s2 == null) ? Ball.EMPTY : s2;
        persistToStorage();
    }

    public void forceIntakeSlotGreen(Telemetry telemetry) {
        if (slots[0] == Ball.EMPTY) slots[0] = Ball.GREEN;
        if (telemetry != null) telemetry.addData("Force", "slot0=GREEN");
    }

    public void forceIntakeSlotPurple(Telemetry telemetry) {
        if (slots[0] == Ball.EMPTY) slots[0] = Ball.PURPLE;
        if (telemetry != null) telemetry.addData("Force", "slot0=PURPLE");
    }

    public void debugAbsAngle(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addData("Spd/ABS", "disabled (incremental)");
        telemetry.addData("Spd/zeroTicks", zeroTicks);
        telemetry.addData("Spd/reverse", REVERSE_MOTOR);
    }

    // =========================
    // ===== MAIN UPDATE ========
    // =========================
    /**
     * Call every loop.
     *
     * @param telemetry DS telemetry
     * @param loader (unused for passive mode; kept for signature compatibility)
     * @param yEdge shoot command edge
     * @param patternTagOverrideFromCode code override 0/21/22/23 or -1
     * @return true if mag full after update
     */
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverrideFromCode) {

        long nowMs = System.currentTimeMillis();

        // Latch shoot request (can be pressed early)
        if (yEdge) shootRequested = true;

        // ===== Panels override: pretend slots contain balls =====
        if (USE_PANELS_SLOT_OVERRIDE) {
            double errToIntake = smallestAngleDiff(normalizeAngle(INTAKE_ANGLE_DEG), getCurrentAngleDeg());
            boolean allow = (!OVERRIDE_ONLY_WHEN_ALIGNED) || (Math.abs(errToIntake) <= ALIGN_TOL_DEG);

            if (allow) {
                int o0 = OVERRIDE_SLOT0;
                int o1 = OVERRIDE_SLOT1;
                int o2 = OVERRIDE_SLOT2;

                boolean s1New = (lastOv1 == 0) && (o1 != 0);
                boolean s2New = (lastOv2 == 0) && (o2 != 0);

                slots[0] = decodeOverride(o0);
                slots[1] = decodeOverride(o1);
                slots[2] = decodeOverride(o2);

                if (OVERRIDE_FORCE_SLOT0_REREAD && (s1New || s2New)) {
                    slot0NeedsReread = true;
                }

                lastOv0 = o0; lastOv1 = o1; lastOv2 = o2;

                if (telemetry != null) telemetry.addData("OVERRIDE", "S0=%d S1=%d S2=%d", o0, o1, o2);
            } else {
                if (telemetry != null) telemetry.addData("OVERRIDE", "enabled but not near intake; ignoring");
            }
        }

        // Apply tag overrides
        int effectiveTag = -1;
        boolean panelsValid = (patternTagOverride == 0 || patternTagOverride == 21 || patternTagOverride == 22 || patternTagOverride == 23);
        boolean codeValid   = (patternTagOverrideFromCode == 0 || patternTagOverrideFromCode == 21 || patternTagOverrideFromCode == 22 || patternTagOverrideFromCode == 23);
        if (panelsValid) effectiveTag = patternTagOverride;
        else if (codeValid) effectiveTag = patternTagOverrideFromCode;
        if (effectiveTag != -1) gameTag = effectiveTag;

        // Update sensing ONLY when aligned at intake and NOT shooting
        boolean aligned = isAlignedAtIntake();
        if (aligned && state != State.SHOOTING) {
            updateSensorsAligned(telemetry);
        }

        // If we become full in IDLE, start positioning to preshoot (but do not shoot yet)
        if (state == State.IDLE && isFull()) {
            planToPreshootFromCurrent();
            state = State.POSITION_TO_PARK_CW;
        }

        // Allow partial mags: if shoot requested in IDLE and we have any ball, go preshoot first
        if (state == State.IDLE && shootRequested && hasAnyBall() && !isFull()) {
            planToPreshootFromCurrent();
            state = State.POSITION_TO_PARK_CW;
        }

        // Reset PID when state changes (prevents windup and “stuck” behavior)
        if (lastState != state) {
            resetDrivePID();
            lastState = state;
        }

        // State machine
        switch (state) {
            case IDLE: {
                ejecting = false;
                // Hold softly at targetAngleDeg
                holdToTargetShortestPath();
                break;
            }

            case POSITION_TO_PARK_CW: {
                ejecting = false;
                boolean done = moveTowardTargetCW(parkWheelAngleDeg);
                if (done) state = State.POSITION_TO_PRESHOOT_CCW;
                break;
            }

            case POSITION_TO_PRESHOOT_CCW: {
                ejecting = false;
                boolean done = moveTowardTargetCCW(preshootWheelAngleDeg);
                if (done) state = State.READY;
                break;
            }

            case READY: {
                ejecting = false;
                holdToTargetShortestPath();

                if (shootRequested) {
                    beginShooting(nowMs);
                    state = State.SHOOTING;
                    // PID reset on next loop due to state change
                }
                break;
            }

            case SHOOTING: {
                ejecting = true;
                boolean done = updateShooting(nowMs);
                if (done) {
                    // passive assumption: everything got shot
                    slots[0] = Ball.EMPTY;
                    slots[1] = Ball.EMPTY;
                    slots[2] = Ball.EMPTY;

                    // Return to intake (SAFE CW)
                    state = State.RETURNING_CW;
                    targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);

                    clearAllSensingCounters();
                    slot0NeedsReread = true;
                }
                break;
            }

            case RETURNING_CW: {
                ejecting = false;
                boolean done = moveTowardTargetCW(normalizeAngle(INTAKE_ANGLE_DEG));
                if (done) state = State.IDLE;
                break;
            }
        }

        persistToStorage();

        // Optional debug telemetry
        if (telemetry != null) {
            telemetry.addData("Spd/state", state);
            telemetry.addData("Spd/aligned", aligned);
            telemetry.addData("Spd/angle", "%.1f", getCurrentAngleDeg());
            telemetry.addData("Spd/target", "%.1f", targetAngleDeg);
            telemetry.addData("Spd/tag", gameTag);
            telemetry.addData("Spd/slots", "%s %s %s", slots[0], slots[1], slots[2]);
            telemetry.addData("Spd/startSlot", selectedStartSlot);
            telemetry.addData("Spd/shootReq", shootRequested);
            telemetry.addData("Spd/pwrReq", "%.2f", dbgLastRequestedPower);
            telemetry.addData("Spd/pwrSet", "%.2f", dbgLastAppliedPower);
            telemetry.addData("Spd/eShort", "%.1f", dbgLastErrShortestDeg);
            telemetry.addData("Spd/eUsed", "%.1f", dbgLastErrUsedDeg);
        }

        return isFull();
    }

    // =========================
    // ===== PLANNING ===========
    // =========================
    private void planToPreshootFromCurrent() {
        selectedStartSlot = chooseStartSlotForCurrentTag();

        double parkWorld = normalizeAngle(PRESHOOT_ANGLE_DEG + GO_PAST_ANGLE_DEG);

        parkWheelAngleDeg     = wheelAngleForPocketAtWorldAngle(selectedStartSlot, parkWorld);
        preshootWheelAngleDeg = wheelAngleForPocketAtWorldAngle(selectedStartSlot, PRESHOOT_ANGLE_DEG);
    }

    private int chooseStartSlotForCurrentTag() {
        int[] candidates = new int[]{0, 1, 2};

        Ball[] desired = getPatternForTag(gameTag);

        double cur = getCurrentAngleDeg();
        double parkWorld = normalizeAngle(PRESHOOT_ANGLE_DEG + GO_PAST_ANGLE_DEG);

        int bestSlot = 0;
        int bestScore = -1;
        double bestCw = Double.MAX_VALUE;

        for (int s : candidates) {
            if (slots[s] == Ball.EMPTY) continue;

            int score = 0;
            if (desired != null) {
                int[] order = EJECT_ORDER[s];
                score += matchScore(desired, order);
            }

            double parkWheel = wheelAngleForPocketAtWorldAngle(s, parkWorld);
            double cw = cwDeltaDeg(cur, parkWheel);

            if (score > bestScore || (score == bestScore && cw < bestCw)) {
                bestScore = score;
                bestCw = cw;
                bestSlot = s;
            }
        }

        if (bestScore < 0) {
            bestSlot = 0;
            bestCw = Double.MAX_VALUE;
            for (int s : candidates) {
                if (slots[s] == Ball.EMPTY) continue;
                double parkWheel = wheelAngleForPocketAtWorldAngle(s, parkWorld);
                double cw = cwDeltaDeg(cur, parkWheel);
                if (cw < bestCw) { bestCw = cw; bestSlot = s; }
            }
        }

        return bestSlot;
    }

    private int matchScore(Ball[] desired, int[] order) {
        int score = 0;
        Ball a0 = slots[order[0]];
        Ball a1 = slots[order[1]];
        Ball a2 = slots[order[2]];

        if (a0 != Ball.EMPTY && desired[0] == a0) score += 4;
        if (a1 != Ball.EMPTY && desired[1] == a1) score += 2;
        if (a2 != Ball.EMPTY && desired[2] == a2) score += 1;

        return score;
    }

    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];
        switch (tag) {
            case 23: seq[0] = Ball.PURPLE; seq[1] = Ball.PURPLE; seq[2] = Ball.GREEN;  return seq;
            case 22: seq[0] = Ball.PURPLE; seq[1] = Ball.GREEN;  seq[2] = Ball.PURPLE; return seq;
            case 21: seq[0] = Ball.GREEN;  seq[1] = Ball.PURPLE; seq[2] = Ball.PURPLE; return seq;
            default: return null;
        }
    }

    private double wheelAngleForPocketAtWorldAngle(int slotIndex, double worldAngleDeg) {
        return normalizeAngle(worldAngleDeg - slotIndex * DEGREES_PER_SLOT);
    }

    // =========================
    // ===== SENSING (aligned) ==
    // =========================
    private void clearAllSensingCounters() {
        slot0PresentFrames = 0;
        slot0ColorFrames = 0;
        slot0LastColorFrame = Ball.EMPTY;

        for (int i = 0; i < SLOT_COUNT; i++) {
            presentFrames[i] = 0;
            colorFrames[i] = 0;
            lastColorFrame[i] = Ball.EMPTY;
        }
    }

    private void updateSensorsAligned(Telemetry telemetry) {
        // Slot1 & Slot2 (Brushland)
        updateBrushlandSlot(1, slot1Present.getState(), slot1Green.getState(), telemetry);
        updateBrushlandSlot(2, slot2Present.getState(), slot2Green.getState(), telemetry);

        // Slot0 needs reread => clear ONCE, then allow frames to accumulate
        if (slot0NeedsReread) {
            slots[0] = Ball.EMPTY;
            slot0PresentFrames = 0;
            slot0ColorFrames = 0;
            slot0LastColorFrame = Ball.EMPTY;
            slot0NeedsReread = false;
        }

        if (slots[0] == Ball.EMPTY) {
            boolean present = slot0PresentThisFrame();
            Ball frameColor = Ball.EMPTY;
            if (present) frameColor = slot0ColorThisFrame();

            if (!present) {
                slot0PresentFrames = 0;
                slot0ColorFrames = 0;
                slot0LastColorFrame = Ball.EMPTY;
            } else {
                slot0PresentFrames++;

                if (frameColor == Ball.GREEN || frameColor == Ball.PURPLE) {
                    if (frameColor == slot0LastColorFrame) slot0ColorFrames++;
                    else { slot0LastColorFrame = frameColor; slot0ColorFrames = 1; }
                } else {
                    slot0LastColorFrame = Ball.EMPTY;
                    slot0ColorFrames = 0;
                }

                if (slot0PresentFrames >= PRESENT_FRAMES_REQUIRED && slot0ColorFrames >= COLOR_FRAMES_REQUIRED) {
                    slots[0] = slot0LastColorFrame;

                    slot0PresentFrames = 0;
                    slot0ColorFrames = 0;
                    slot0LastColorFrame = Ball.EMPTY;

                    if (telemetry != null) telemetry.addData("Slot0", "Committed %s", slots[0]);
                }
            }
        }
    }

    private void updateBrushlandSlot(int slotIndex, boolean presentRaw, boolean greenRaw, Telemetry telemetry) {
        if (slotIndex != 1 && slotIndex != 2) return;
        if (slots[slotIndex] != Ball.EMPTY) return; // latch only

        boolean present = presentRaw; // active-high
        Ball frameColor = Ball.EMPTY;

        if (present) frameColor = greenRaw ? Ball.GREEN : Ball.PURPLE;

        if (!present) {
            presentFrames[slotIndex] = 0;
            colorFrames[slotIndex] = 0;
            lastColorFrame[slotIndex] = Ball.EMPTY;
            return;
        }

        presentFrames[slotIndex]++;

        if (frameColor == Ball.GREEN || frameColor == Ball.PURPLE) {
            if (frameColor == lastColorFrame[slotIndex]) colorFrames[slotIndex]++;
            else { lastColorFrame[slotIndex] = frameColor; colorFrames[slotIndex] = 1; }
        } else {
            lastColorFrame[slotIndex] = Ball.EMPTY;
            colorFrames[slotIndex] = 0;
        }

        if (presentFrames[slotIndex] >= PRESENT_FRAMES_REQUIRED && colorFrames[slotIndex] >= COLOR_FRAMES_REQUIRED) {
            slots[slotIndex] = lastColorFrame[slotIndex];

            // IMPORTANT RULE: if slot1 or slot2 gets a ball, slot0 must be reread
            slot0NeedsReread = true;

            presentFrames[slotIndex] = 0;
            colorFrames[slotIndex] = 0;
            lastColorFrame[slotIndex] = Ball.EMPTY;

            if (telemetry != null) telemetry.addData("Slot" + slotIndex, "Committed %s (slot0 reread)", slots[slotIndex]);
        }
    }

    private boolean slot0PresentThisFrame() {
        double d1 = slot0Color1.getDistance(DistanceUnit.CM);
        double d2 = slot0Color2.getDistance(DistanceUnit.CM);

        boolean p1 = !Double.isNaN(d1) && d1 <= SLOT0_DIST_THRESH_CM;
        boolean p2 = !Double.isNaN(d2) && d2 <= SLOT0_DIST_THRESH_CM;
        return p1 || p2;
    }

    private Ball slot0ColorThisFrame() {
        double d1 = slot0Color1.getDistance(DistanceUnit.CM);
        double d2 = slot0Color2.getDistance(DistanceUnit.CM);

        boolean p1 = !Double.isNaN(d1) && d1 <= SLOT0_DIST_THRESH_CM;
        boolean p2 = !Double.isNaN(d2) && d2 <= SLOT0_DIST_THRESH_CM;

        if (!p1 && !p2) return Ball.EMPTY;

        RevColorSensorV3 chosen;
        if (p1 && p2) chosen = (d1 <= d2) ? slot0Color1 : slot0Color2;
        else chosen = p1 ? slot0Color1 : slot0Color2;

        RawColor raw = classifyColor(chosen);
        if (raw == RawColor.GREEN) return Ball.GREEN;
        if (raw == RawColor.PURPLE) return Ball.PURPLE;
        return Ball.UNKNOWN;
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

    private boolean isAlignedAtIntake() {
        double cur = getCurrentAngleDeg();
        double err = smallestAngleDiff(normalizeAngle(INTAKE_ANGLE_DEG), cur);
        return Math.abs(err) <= ALIGN_TOL_DEG;
    }

    // =========================
    // ===== MOTION CONTROL =====
    // =========================
    private void setMotorPower(double pwr) {
        dbgLastRequestedPower = pwr;

        double out = REVERSE_MOTOR ? -pwr : pwr;
        out = Range.clip(out, -1.0, 1.0);

        dbgLastAppliedPower = out;
        motor.setPower(out);
    }

    private double computeErrorDeg(double targetDeg, ErrorMode mode) {
        double cur = getCurrentAngleDeg();
        double target = normalizeAngle(targetDeg);

        double eShort = smallestAngleDiff(target, cur); // [-180,180]
        dbgLastErrShortestDeg = eShort;

        if (mode == ErrorMode.SHORTEST) return eShort;

        if (mode == ErrorMode.PREFER_CW) {
            // Prefer CW, unless we're within overshoot-correct window
            if (eShort < -OVERSHOOT_CORRECT_MAX_DEG) return eShort + 360.0;
            return eShort;
        }

        // PREFER_CCW
        if (eShort > OVERSHOOT_CORRECT_MAX_DEG) return eShort - 360.0;
        return eShort;
    }

    private boolean driveToTarget(double targetDeg, ErrorMode mode, double maxPower) {
        double cur = getCurrentAngleDeg();
        double target = normalizeAngle(targetDeg);

        // DONE check ALWAYS uses shortest error so it can't get stuck
        double eShort = smallestAngleDiff(target, cur);
        if (Math.abs(eShort) <= MOVE_DONE_TOL_DEG) {
            targetAngleDeg = target;
            setMotorPower(0.0);
            resetDrivePID();
            dbgLastErrUsedDeg = 0.0;
            return true;
        }

        double e = computeErrorDeg(target, mode);
        dbgLastErrUsedDeg = e;

        long nowNs = System.nanoTime();
        double dt = (nowNs - pidLastNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        pidLastNs = nowNs;

        pidIntegral += e * dt;
        pidIntegral = Range.clip(pidIntegral, -I_CLAMP, I_CLAMP);

        double deriv = (e - pidLastErrDeg) / dt;
        pidLastErrDeg = e;

        double ff = kF * Math.signum(e);

        double out = kP * e + kI * pidIntegral + kD * deriv + ff;
        out = Range.clip(out, -maxPower, maxPower);

        targetAngleDeg = target;
        setMotorPower(out);
        return false;
    }

    /** Prefer CW (safe), allow small backtrack for overshoot. */
    private boolean moveTowardTargetCW(double targetDeg) {
        return driveToTarget(targetDeg, ErrorMode.PREFER_CW, MOVE_MAX_POWER);
    }

    /** Prefer CCW (eject direction), allow small backtrack for overshoot. */
    private boolean moveTowardTargetCCW(double targetDeg) {
        return driveToTarget(targetDeg, ErrorMode.PREFER_CCW, MOVE_MAX_POWER);
    }

    /** Shortest-path holding at targetAngleDeg. */
    private void holdToTargetShortestPath() {
        driveToTarget(targetAngleDeg, ErrorMode.SHORTEST, HOLD_MAX_POWER);
    }

    // =========================
    // ===== SHOOTING ===========
    // =========================
    private void beginShooting(long nowMs) {
        shootRequested = false;

        shootStartMs = nowMs;
        shootMovedDeg = 0.0;
        shootLastTicks = motor.getCurrentPosition();

        // Open-loop CCW in updateShooting()
    }

    private boolean updateShooting(long nowMs) {
        if (nowMs - shootStartMs > SHOOT_TIMEOUT_MS) {
            setMotorPower(0.0);
            return true;
        }

        int curTicks = motor.getCurrentPosition();
        int dTicks = curTicks - shootLastTicks;
        shootLastTicks = curTicks;

        shootMovedDeg += Math.abs(dTicks) * DEG_PER_TICK;

        double pwr = (nowMs - shootStartMs <= SHOOT_HIGH_MS) ? SHOOT_POWER_HIGH : SHOOT_POWER_LOW;

        // CCW is eject direction => negative power in angle-space
        setMotorPower(-pwr);

        if (shootMovedDeg >= SHOOT_DEG) {
            setMotorPower(0.0);
            return true;
        }

        return false;
    }

    // =========================
    // ===== ANGLE MATH =========
    // =========================
    private double normalizeAngle(double a) {
        return (a % 360.0 + 360.0) % 360.0;
    }

    private double smallestAngleDiff(double target, double current) {
        double diff = normalizeAngle(target) - normalizeAngle(current);
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    /** CW delta from 'from' to 'to' in [0,360). */
    private double cwDeltaDeg(double fromDeg, double toDeg) {
        double f = normalizeAngle(fromDeg);
        double t = normalizeAngle(toDeg);
        return (t - f + 360.0) % 360.0;
    }

    private double ticksToAngle(int ticks) {
        // If REVERSE_MOTOR flips, motor ticks move opposite direction for the same physical motion.
        int rel = ticks - zeroTicks;
        int signedRel = REVERSE_MOTOR ? -rel : rel;

        double revs = signedRel / TICKS_PER_REV;
        return normalizeAngle(INTAKE_ANGLE_DEG + revs * 360.0);
    }

    private int angleToTicks(double angleDeg) {
        double norm = normalizeAngle(angleDeg - INTAKE_ANGLE_DEG);
        double revs = norm / 360.0;

        int signedRel = (int) Math.round(revs * TICKS_PER_REV);
        int rel = REVERSE_MOTOR ? -signedRel : signedRel;
        return zeroTicks + rel;
    }
}

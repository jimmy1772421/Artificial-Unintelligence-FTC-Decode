package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class SpindexerSubsystem {

    private ShooterSubsystem shooter;

    public enum Ball {
        EMPTY,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    // ==== MOTOR / GEOMETRY CONSTANTS ====

    // goBILDA 435 rpm YJ integrated encoder
    private static final double TICKS_PER_REV = 383.6;
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;   // "home" angle
    private static final double LOAD_ANGLE   = 180.0;
    private static final int SLOT_COUNT = 3;
    private static final int TOLERANCE_TICKS = 10;
    private static final double MOVE_POWER = 0.5;

    // Analog abs encoder
    private static final double ABS_VREF = 3.3; // REV analog reference

    // Raw abs angle (deg) when SLOT 0 is perfectly at intake (you measured this)
    // i.e. absRaw == 260.7°  <=>  internal angle == 0°
    private static final double ABS_MECH_OFFSET_DEG = 260.8;

    // ==== HARDWARE ====

    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    // encoder value for angle 0° (intake) when slot 0 is at intake
    private int zeroTicks = 0;

    // which slot index is currently "at intake" (0,1,2)
    private int intakeIndex = 0;

    // pattern first eject index (used by some helper methods)
    private int ejectStartIndex = 0;

    private final Ball[] slots = new Ball[SLOT_COUNT];

    // Auto-intake state
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;

    // Eject sequence state (moved from TeleOp)
    private boolean ejecting = false;
    private int ejectSlotIndex = 0;
    private long ejectPhaseTime = 0;
    private int ejectPhase = 0; // 0 = find/rotate, 1 = wait before loader, 2 = wait after loader

    //shooter
    private boolean isOn = false;

    // ==== AprilTag / pattern state ====
    // 23 = P, P, G
    // 22 = P, G, P
    // 21 = G, P, P
    //  0 = no valid tag -> "fastest possible" logic
    private int gameTag = 0;
    // index into the 3-shot pattern (0,1,2) while we are in an eject sequence
    private int patternStep = 0;

    public void setGameTag(int tag) {
        this.gameTag = tag;
    }

    public int getGameTag() {
        return gameTag;
    }

    private static final long WAIT_BEFORE_LOADER_MS = 300;
    private static final long WAIT_AFTER_LOADER_MS  = 500;



    public SpindexerSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        absEncoder = hardwareMap.get(AnalogInput.class, "spindexerAbs");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
        }

        // One-shot auto-zero at startup using absolute encoder
        autoZeroFromAbs();

        homeToIntake();
    }

    // ===== Absolute encoder helpers =====

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
    }

    // ===== Angle / encoder helpers =====

    private double ticksToAngle(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        double angle = revs * 360.0;
        return normalizeAngle(angle);
    }

    /**
     * Angle estimated from motor encoder, in degrees [0, 360).
     * 0° = slot 0 at intake (raw abs ≈ 260.7°).
     */
    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
    }

    /**
     * Compute the motor tick target for a given angle so that
     * the motor takes the SHORTEST path from its current tick position.
     */
    private int shortestTicksToAngle(double angleDeg) {
        double norm = normalizeAngle(angleDeg);

        // "Canonical" tick for this angle (somewhere on the infinite shaft)
        double desiredRevs = norm / 360.0;
        int baseTicks = zeroTicks + (int)Math.round(desiredRevs * TICKS_PER_REV);

        int current = motor.getCurrentPosition();
        int revTicks = (int)Math.round(TICKS_PER_REV);

        int diff = baseTicks - current;  // how far we'd move if we used baseTicks directly

        // Shift by an integer number of full revs so |diff'| is minimized
        int k = (int)Math.round((double)diff / revTicks);

        int bestTarget = baseTicks - k * revTicks;  // new target with minimal travel
        return bestTarget;
    }

    private void runToAngleBlocking(double angleDeg, double power) {
        int target = shortestTicksToAngle(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        long startTime = System.currentTimeMillis();
        long timeoutMs = 700;  // tweak as needed

        while (motor.isBusy()
                && (System.currentTimeMillis() - startTime) < timeoutMs) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                break;
            }
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // === PUBLIC ANGLE API ===

    /** Move to an absolute angle (0, 30, 180, etc.) blocking up to timeout. */
    public void moveToAngleBlocking(double angleDeg) {
        runToAngleBlocking(angleDeg, MOVE_POWER);
    }

    public void moveToAngleBlocking(double angleDeg, double power) {
        runToAngleBlocking(angleDeg, power);
    }

    /** Non-blocking move to an absolute angle. */
    public void moveToAngleAsync(double angleDeg, double power) {
        int target = shortestTicksToAngle(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private void moveSlotToIntake(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex);
        runToAngleBlocking(angle, power);
        intakeIndex = slotIndex;
    }

    private void moveSlotToLoad(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        runToAngleBlocking(angle, power);
    }

    // ===== Color / ball handling =====

// ===== Color / ball handling =====



    /**
     * Classify color from a single REV color sensor (GREEN / PURPLE / UNKNOWN).
     */
// Returns true if either sensor sees something close
    private boolean isBallPresent() {
        final double THRESH_CM = 3.0; // a bit tighter than 5.0

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        return present1 || present2;
    }

    // Classify color from a single sensor
    private Ball classifyColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int sum = r + g + b;
        if (sum < 60) {
            return Ball.UNKNOWN;
        }

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        // Strong green dominance → GREEN
        if (gn > rn + 0.10 && gn > bn + 0.10) {
            return Ball.GREEN;
        }

        // Anything else we treat as PURPLE
        return Ball.PURPLE;
    }

    // Use BOTH sensors:
// - If neither sees a close object → UNKNOWN
// - If both see the ball → use the closer one for color
// - If only one sees it → use that one
    private Ball detectBallColor() {
        final double THRESH_CM = 3.0;

        double d1 = intakeColor.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean present1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean present2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        if (!present1 && !present2) {
            return Ball.UNKNOWN;
        }

        RevColorSensorV3 chosenSensor;

        if (present1 && present2) {
            // Use whichever is closer to the ball
            chosenSensor = (d1 <= d2) ? intakeColor : intakeColor2;
        } else if (present1) {
            chosenSensor = intakeColor;
        } else {
            chosenSensor = intakeColor2;
        }

        return classifyColor(chosenSensor);
    }





    /**
     * Manual "take one" from the intake.
     *
     * @param telemetry FTC telemetry
     * @param tag       Limelight / AprilTag pattern ID:
     *                  23 = P,P,G; 22 = P,G,P; 21 = G,P,P; 0 = no valid tag
     */
    public void intakeOne(Telemetry telemetry, int tag) {
        // Remember the last tag we saw so the eject logic can use it
        this.gameTag = tag;

        moveSlotToIntake(intakeIndex, MOVE_POWER);

        Ball color = detectBallColor();
        slots[intakeIndex] = color;

        telemetry.addData("Intake", "Slot %d = %s (tag=%d)", intakeIndex, color, gameTag);

        pendingAutoRotate = true;
        autoRotateTimeMs = System.currentTimeMillis() + 100;
    }

    // ===== Pattern helpers (AprilTag → desired 3-shot color order) =====

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

            double targetAngle =
                    slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
            double diff = Math.abs(
                    smallestAngleDiff(targetAngle, currentAngle)
            );

            if (diff < bestDiff) {
                bestDiff = diff;
                bestSlot = i;
            }
        }

        return bestSlot;
    }

    /**
     * Choose which slot to eject next.
     *
     * If we have a valid tag (21/22/23), we follow the color pattern:
     *   step 0,1,2 → desired color from getPatternForTag(tag)
     * For each desired color we choose the *nearest* slot of that color.
     * If no slot of that color exists, fall back to "fastest non-empty" logic.
     *
     * If tag is 0 / invalid → always "fastest non-empty".
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

                double targetAngle =
                        slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
                double diff = Math.abs(
                        smallestAngleDiff(targetAngle, currentAngle)
                );

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

    /**
     * Returns true if the slot indexed by intakeIndex is actually
     * lined up at the intake position (within a small angle tolerance).
     */
    private boolean isIntakeSlotAtIntakePosition() {
        double desiredAngle = slotCenterAngleAtIntake(intakeIndex); // 0°, 120°, or 240°
        double currentAngle = getCurrentAngleDeg();

        double diff = smallestAngleDiff(currentAngle, desiredAngle);
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;  // same style as isAtMid()
        return Math.abs(diff) <= tolDeg;
    }



    // ===== MAIN UPDATE: auto-intake + ejection =====
    //
    // Call this every loop from TeleOp.
    //  - telemetry: for debug prints
    //  - loader:    so we can start loader cycles during eject
    //  - yEdge:     true only on rising edge of Y
    public boolean update(Telemetry telemetry,
                       LoaderSubsystem loader,
                       boolean yEdge) {

        // --- Handle eject button press (start sequence if we have balls) ---
        if (yEdge && !ejecting && hasAnyBall()) {
            ejecting = true;
            ejectPhase = 0;
            patternStep = 0;  // start from the first color in the pattern
        }



// --- Auto-intake (only when a slot is actually at intake) ---
        if (!isFull()) {
            boolean atIntakePose =
                    isIntakeSlotAtIntakePosition()
                            && !ejecting               // don’t intake while shooting
                            && !pendingAutoRotate      // not in the middle of auto-rotate scheduling
                            && !motor.isBusy();        // motor not currently moving

            boolean ballPresent = false;

            if (atIntakePose) {
                ballPresent = isBallPresent();   // uses both sensors as OR
            }

            if (ballPresent && !lastBallPresent && slots[intakeIndex] == Ball.EMPTY) {
                Ball color = detectBallColor();  // also uses both sensors
                slots[intakeIndex] = color;
                telemetry.addData("AutoIntake", "Slot %d = %s", intakeIndex, color);

                pendingAutoRotate = true;
                autoRotateTimeMs = System.currentTimeMillis() + 100;
            }

            lastBallPresent = ballPresent;
        }


        // --- Pending auto-rotate (non-blocking, shortest-path) ---
        if (pendingAutoRotate && System.currentTimeMillis() >= autoRotateTimeMs) {
            if (!motor.isBusy()) {  // Only start if motor is free
                pendingAutoRotate = false;
                int nextIndex = (intakeIndex + 1) % SLOT_COUNT;

                double angle = slotCenterAngleAtIntake(nextIndex);
                int target = shortestTicksToAngle(angle);
                motor.setTargetPosition(target);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(MOVE_POWER);

                intakeIndex = nextIndex;
            }
        }

        // --- Eject sequence state machine (moved from TeleOp) ---
        if (ejecting) {
            long now = System.currentTimeMillis();

            if (ejectPhase == 0) {
                // Decide which slot we want to shoot next
                if (!hasAnyBall()) {
                    // Nothing left to shoot
                    homeToIntake();
                    ejecting = false;
                } else {
                    int nextSlot = selectNextEjectSlotIndex();

                    if (nextSlot < 0 || nextSlot >= SLOT_COUNT) {
                        // No valid slot found, bail out safely
                        homeToIntake();
                        ejecting = false;
                    } else {
                        ejectSlotIndex = nextSlot;

                        // Rotate this slot to LOAD (blocking move with timeout)
                        moveSlotToLoadBlocking(ejectSlotIndex);
                        ejectPhaseTime = now + WAIT_BEFORE_LOADER_MS;
                        ejectPhase = 1;
                    }
                }
            } else if (ejectPhase == 1) {
                // After 0.1s at LOAD, fire the loader
                if (now >= ejectPhaseTime) {
                    loader.startCycle();
                    ejectPhaseTime = now + WAIT_AFTER_LOADER_MS;
                    ejectPhase = 2;
                }
            } else if (ejectPhase == 2) {
                // After extra 0.1s, mark slot empty and either shoot next or finish
                if (now >= ejectPhaseTime) {
                    clearSlot(ejectSlotIndex);

                    // If we were using a pattern, advance to the next color step
                    Ball[] pattern = getPatternForTag(gameTag);
                    if (pattern != null) {
                        patternStep++;
                    }

                    boolean usingPattern = (pattern != null);

                    // Stop if:
                    //  - no balls are left, OR
                    //  - we were following a pattern and we've done all 3 shots
                    if (!hasAnyBall() ||
                            (usingPattern && patternStep >= pattern.length)) {
                        homeToIntake();
                        ejecting = false;
                    } else {
                        ejectPhase = 0; // continue to next shot
                    }
                }
            }
        }



        return isFull();
    }

    // We "have three balls" if all slots are non-EMPTY.
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


    public void moveSlotToLoadBlocking(int slotIndex) {
        moveSlotToLoad(slotIndex, MOVE_POWER);
    }

    public void rezeroHere() {
        zeroTicks = motor.getCurrentPosition();
        intakeIndex = 0;
    }

    public boolean isAtMid() {
        double desiredAngle = slotCenterAngleAtIntake(ejectStartIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        double currentAngle = getCurrentAngleDeg();
        double diff = smallestAngleDiff(currentAngle, normalizeAngle(desiredAngle));
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV;
        return Math.abs(diff) < tolDeg;
    }

    // ==== Debug getters for telemetry ====

    public Ball[] getSlots() {
        return slots;
    }

    public int getEncoder() {
        return motor.getCurrentPosition();
    }

    public int getTarget() {
        return motor.getTargetPosition();
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

    // Which slot the code thinks is at the INTAKE position
    public int getIntakeSlotIndex() {
        return intakeIndex;
    }

    // Are we in the middle of a pending auto-rotate (from auto-intake)?
    public boolean isAutoRotating() {
        return pendingAutoRotate;
    }

    public boolean isEjecting() {
        return ejecting;
    }

    // Convenience: move back so slot 0 is at intake
    public void homeToIntake() {
        moveSlotToIntake(0, MOVE_POWER);
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
}

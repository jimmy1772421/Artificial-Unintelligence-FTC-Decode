package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpindexerSubsystem {

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
    private static final double ABS_MECH_OFFSET_DEG = 260.7;

    // ==== HARDWARE ====

    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor;
    private final AnalogInput absEncoder;

    // encoder value for angle 0° (intake) when slot 0 is at intake
    private int zeroTicks = 0;

    // which slot index is currently "at intake" (0,1,2)
    private int intakeIndex = 0;

    // pattern first eject index
    private int ejectStartIndex = 0;

    private final Ball[] slots = new Ball[SLOT_COUNT];

    // Auto-intake state
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        absEncoder = hardwareMap.get(AnalogInput.class, "spindexerAbs"); // must match config name

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
        }

        // One-shot auto-zero at startup using absolute encoder
        autoZeroFromAbs();
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

        // ticksToAngle(currentTicks) should return internalAngle, so:
        // internalAngle = (currentTicks - zeroTicks) * 360 / TICKS_PER_REV
        // => zeroTicks = currentTicks - internalTicks
        zeroTicks = currentTicks - internalTicks;

        // Figure out which slot is closest to intake (internalAngle 0,120,240)
        double slotIndexF = internalAngle / DEGREES_PER_SLOT; // 0..3
        int nearestIndex = (int) Math.round(slotIndexF) % SLOT_COUNT;
        if (nearestIndex < 0) nearestIndex += SLOT_COUNT;
        intakeIndex = nearestIndex;
    }

    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0; // wrap to [-180, 180)
        return diff;
    }

    // ===== Angle / encoder helpers =====

    private int angleToTicks(double angleDeg) {
        double norm = normalizeAngle(angleDeg);
        double revs = norm / 360.0;
        return zeroTicks + (int) Math.round(revs * TICKS_PER_REV);
    }

    private double ticksToAngle(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        double angle = revs * 360.0;
        angle = normalizeAngle(angle);
        return angle;
    }

    /**
     * Angle estimated from motor encoder, in degrees [0, 360).
     * 0° = slot 0 at intake (raw abs ≈ 260.7°).
     */
    public double getCurrentAngleDeg() {
        return ticksToAngle(motor.getCurrentPosition());
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
    private void moveSlotToIntake(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex); // 0, 120, 240
        runToAngleBlocking(angle, power);                  // shortest path between slots
        intakeIndex = slotIndex;
    }

    private void moveSlotToLoad(int slotIndex, double power) {
        double angle = slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        runToAngleBlocking(angle, power);                  // also shortest path
    }


    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }


    // ===== Color / ball handling =====

    private Ball detectBallColor() {
        // Check distance first: only detect if a ball is "close"
        double distCm = intakeColor.getDistance(DistanceUnit.CM);
        if (distCm > 2.0) {
            return Ball.UNKNOWN; // no ball close enough
        }

        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        int sum = r + g + b;
        if (sum < 60) {
            return Ball.UNKNOWN;
        }

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        if (gn > rn + 0.10 && gn > bn + 0.10) {
            return Ball.GREEN;
        }

        return Ball.PURPLE;
    }

    public void intakeOne(Telemetry telemetry) {
        moveSlotToIntake(intakeIndex, MOVE_POWER);

        Ball color = detectBallColor();
        slots[intakeIndex] = color;

        telemetry.addData("Intake", "Slot %d = %s", intakeIndex, color);

        pendingAutoRotate = true;
        autoRotateTimeMs = System.currentTimeMillis() + 100;
    }

    public void update(Telemetry telemetry) {
        if (!isFull()) {
            double distCm = intakeColor.getDistance(DistanceUnit.CM);
            boolean ballPresent = distCm <= 2.0;

            if (ballPresent && !lastBallPresent && slots[intakeIndex] == Ball.EMPTY) {
                Ball color = detectBallColor();
                slots[intakeIndex] = color;
                telemetry.addData("AutoIntake", "Slot %d = %s", intakeIndex, color);

                pendingAutoRotate = true;
                autoRotateTimeMs = System.currentTimeMillis() + 100;
            }

            lastBallPresent = ballPresent;
        }

        if (pendingAutoRotate && System.currentTimeMillis() >= autoRotateTimeMs) {
            if (!motor.isBusy()) {
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
    }

    public boolean hasThreeBalls() {
        for (Ball b : slots) {
            if (b == Ball.EMPTY) return false;
        }
        return true;
    }

    public String getGamePattern() {
        return "2P + 1G";
    }

    public boolean prepareFirstEjectByPattern(Telemetry telemetry) {
        if (!hasThreeBalls()) {
            telemetry.addLine("Not 3 balls yet; can't prep eject.");
            return false;
        }

        int bestIndex = -1;
        for (int i = 0; i < SLOT_COUNT; i++) {
            Ball a = slots[i];
            Ball b = slots[(i + 1) % SLOT_COUNT];
            Ball c = slots[(i + 2) % SLOT_COUNT];

            if (a == Ball.PURPLE && b == Ball.GREEN && c == Ball.PURPLE) {
                bestIndex = i;
                break;
            }
        }

        if (bestIndex == -1) {
            bestIndex = intakeIndex;
            telemetry.addLine("Pattern not perfect; starting eject from intakeIndex.");
        } else {
            telemetry.addData("Pattern", "Using P-G-P starting at slot %d", bestIndex);
        }

        ejectStartIndex = bestIndex;
        moveSlotToLoad(ejectStartIndex, MOVE_POWER);
        telemetry.addData("Prepared", "Slot %d at LOAD (180°)", ejectStartIndex);

        return true;
    }

    public void ejectAllByPattern(Telemetry telemetry) {
        for (int i = 0; i < SLOT_COUNT; i++) {
            int slotIndex = (ejectStartIndex + i) % SLOT_COUNT;

            moveSlotToLoad(slotIndex, MOVE_POWER);
            telemetry.addData("Eject", "Slot %d at LOAD", slotIndex);
            telemetry.update();

            slots[slotIndex] = Ball.EMPTY;

            try {
                Thread.sleep(150);
            } catch (InterruptedException ignored) {
            }
        }

        moveSlotToIntake(0, MOVE_POWER);
        intakeIndex = 0;
    }

    public void rezeroHere() {
        zeroTicks = motor.getCurrentPosition();
        intakeIndex = 0;
    }

    public boolean isAtMid() {
        double desiredAngle = slotCenterAngleAtIntake(ejectStartIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        double currentAngle = getCurrentAngleDeg();
        double diff = smallestAngleDiff(currentAngle, normalizeAngle(desiredAngle));
        // translate angle tolerance to ticks, or just use a degree threshold:
        double tolDeg = 360.0 * TOLERANCE_TICKS / TICKS_PER_REV; // same as your tick tolerance
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

    public void moveSlotToLoadBlocking(int slotIndex) {
        moveSlotToLoad(slotIndex, MOVE_POWER);
    }

    public int getIntakeSlotIndex() {
        return intakeIndex;
    }

    public boolean isAutoRotating() {
        return pendingAutoRotate;
    }

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
        // k ≈ diff / revTicks, rounded to nearest integer
        int k = (int)Math.round((double)diff / revTicks);

        int bestTarget = baseTicks - k * revTicks;  // new target with minimal travel
        return bestTarget;
    }

}

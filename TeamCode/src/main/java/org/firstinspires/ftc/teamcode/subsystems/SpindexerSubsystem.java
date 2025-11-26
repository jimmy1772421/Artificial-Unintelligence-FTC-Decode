package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import android.graphics.Color;



import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpindexerSubsystem {

    public enum Ball {
        EMPTY,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    // ==== MOTOR / GEOMETRY CONSTANTS ====

    // goBILDA 435 rpm YJ
    private static final double TICKS_PER_REV = 383.6;
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;
    private static final double LOAD_ANGLE = 180.0;
    private static final int SLOT_COUNT = 3;
    private static final int TOLERANCE_TICKS = 10;
    private static final double MOVE_POWER = 0.5;

    // Analog abs encoder
    private static final double ABS_VREF = 3.3; // REV analog reference

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

        // Auto-zero at startup using absolute encoder
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

    /**
     * Use the absolute encoder to figure out which slot is at intake,
     * and set zeroTicks so angleToTicks(slotCenterAngle) == current motor pos.
     */
    private void autoZeroFromAbs() {
        double angle = getAbsAngleDeg();

        // Slot centers: 0, 120, 240
        double[] centers = {0.0, 120.0, 240.0};

        int bestIndex = 0;
        double bestError = 9999;

        for (int i = 0; i < centers.length; i++) {
            double err = smallestAngleDiff(angle, centers[i]);
            if (Math.abs(err) < bestError) {
                bestError = Math.abs(err);
                bestIndex = i;
            }
        }

        intakeIndex = bestIndex;
        double bestCenter = centers[bestIndex];

        // We want angleToTicks(bestCenter) == current motor encoder reading
        int currentTicks = motor.getCurrentPosition();
        int expectedOffset = (int) Math.round((bestCenter / 360.0) * TICKS_PER_REV);
        zeroTicks = currentTicks - expectedOffset;
    }

    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0; // wrap to [-180, 180)
        return diff;
    }

    // ===== Angle / encoder helpers =====

    private int angleToTicks(double angleDeg) {
        double revs = angleDeg / 360.0;
        return zeroTicks + (int) Math.round(revs * TICKS_PER_REV);
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private void runToAngleBlocking(double angleDeg, double power) {
        int target = angleToTicks(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while (motor.isBusy()) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                break;
            }
        }

        motor.setPower(0);
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

    private Ball detectBallColor() {
        // Raw RGB from REV Color Sensor V3
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        // If everything is very dark, treat as no ball
        if (r + g + b < 50) {
            return Ball.UNKNOWN;
        }

        // Convert to HSV using Android's Color utility
        float[] hsv = new float[3];
        // Color.rgb expects 0–255, sensor values are usually in that ballpark already
        int rgb = Color.rgb(r, g, b);
        Color.colorToHSV(rgb, hsv);

        float hue = hsv[0];   // 0..360
        float sat = hsv[1];   // 0..1
        float val = hsv[2];   // 0..1

        // Filter out very dark or desaturated readings
        if (val < 0.1f) return Ball.UNKNOWN;
        if (sat < 0.3f) return Ball.UNKNOWN;

        // ---- Initial ranges (tune after testing!) ----
        // Green ~ 80–160°
        if (hue >= 80f && hue <= 160f) {
            return Ball.GREEN;
        }

        // Purple / violet ~ 260–320°
        if (hue >= 260f && hue <= 320f) {
            return Ball.PURPLE;
        }


        return Ball.UNKNOWN;
    }


    /**
     * Called when X is pressed:
     *  1) ensure a slot is at intake
     *  2) read color into that slot
     *  3) rotate +120° so the next slot is at intake
     */
    public void intakeOne(Telemetry telemetry) {
        // 1. Make sure current slot is aligned at intake
        moveSlotToIntake(intakeIndex, MOVE_POWER);

        // 2. Read color
        Ball color = detectBallColor();
        slots[intakeIndex] = color;

        telemetry.addData("Intake", "Slot %d = %s", intakeIndex, color);

        // 3. Advance to next slot at intake
        int nextIndex = (intakeIndex + 1) % SLOT_COUNT;
        moveSlotToIntake(nextIndex, MOVE_POWER);
    }

    // We "have three balls" if all slots are non-EMPTY.
    public boolean hasThreeBalls() {
        for (Ball b : slots) {
            if (b == Ball.EMPTY) return false;
        }
        return true;
    }

    // Game pattern string (for telemetry only)
    public String getGamePattern() {
        return "2P + 1G";
    }

    /**
     * Pick a first eject slot that makes sense for the pattern.
     * For now: prefer GREEN in the middle (P-G-P), but if not, just pick intakeIndex.
     * Moves that slot to the LOAD position (180° from intake).
     */
    public boolean prepareFirstEjectByPattern(Telemetry telemetry) {
        if (!hasThreeBalls()) {
            telemetry.addLine("Not 3 balls yet; can't prep eject.");
            return false;
        }

        // Try to find a P-G-P sequence (cyclic)
        int bestIndex = -1;
        for (int i = 0; i < SLOT_COUNT; i++) {
            Ball a = slots[i];
            Ball b = slots[(i + 1) % SLOT_COUNT];
            Ball c = slots[(i + 2) % SLOT_COUNT];

            if (a == Ball.PURPLE && b == Ball.GREEN && c == Ball.PURPLE) {
                // eject order: P (i), G (i+1), P (i+2)
                bestIndex = i;
                break;
            }
        }

        if (bestIndex == -1) {
            // Fall back: just start from whatever slot is currently at intake
            bestIndex = intakeIndex;
            telemetry.addLine("Pattern not perfect; starting eject from intakeIndex.");
        } else {
            telemetry.addData("Pattern", "Using P-G-P starting at slot %d", bestIndex);
        }

        ejectStartIndex = bestIndex;

        // Move that slot to LOAD position (180° from intake)
        moveSlotToLoad(ejectStartIndex, MOVE_POWER);
        telemetry.addData("Prepared", "Slot %d at LOAD (180°)", ejectStartIndex);

        return true;
    }

    /**
     * Eject all 3 in pattern order, always putting the current eject slot
     * at LOAD (180°). Afterward, slot 0 is returned to intake.
     */
    public void ejectAllByPattern(Telemetry telemetry) {
        for (int i = 0; i < SLOT_COUNT; i++) {
            int slotIndex = (ejectStartIndex + i) % SLOT_COUNT;

            // Move slotIndex to load position
            moveSlotToLoad(slotIndex, MOVE_POWER);
            telemetry.addData("Eject", "Slot %d at LOAD", slotIndex);
            telemetry.update();

            // Here, your loader mechanism would fire.
            // We just mark it empty:
            slots[slotIndex] = Ball.EMPTY;

            try {
                Thread.sleep(150); // small pause for your loader, tune as needed
            } catch (InterruptedException ignored) {
                // ignore
            }
        }

        // After all ejected, rotate back so that slot 0 is at intake
        moveSlotToIntake(0, MOVE_POWER);
        intakeIndex = 0;
    }

    public void rezeroHere() {
        zeroTicks = motor.getCurrentPosition();
        intakeIndex = 0;
    }

    public boolean isAtMid() {
        // "Mid" here = LOAD position for the currently selected ejectStartIndex
        double angle = slotCenterAngleAtIntake(ejectStartIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        int target = angleToTicks(angle);
        return Math.abs(motor.getCurrentPosition() - target) < TOLERANCE_TICKS;
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
}

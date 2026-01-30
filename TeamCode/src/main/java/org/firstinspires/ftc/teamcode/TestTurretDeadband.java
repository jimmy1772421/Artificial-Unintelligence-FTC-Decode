package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemAbsoluteServos;

/**
 * Purpose:
 * 1) Measure CR servo deadband / breakaway power (smallest |power| that moves)
 * 2) Check neutral offset (does it drift at power=0?)
 * 3) Verify sign/direction quickly
 *
 * Controls:
 *  - GP1 Dpad Up/Down : increase/decrease commanded power by STEP
 *  - GP1 A           : toggle applyPower (on/off)
 *  - GP1 B           : set power = 0 (stop) and keep applyPower ON
 *  - GP1 X           : reverse sign (flip direction) (software only)
 *  - GP1 Y           : reset "moved?" baseline (stores current angle as baseline)
 *
 * Readouts:
 *  - Current angle, baseline angle, delta angle
 *  - Commanded power, applied power
 *  - "MOVING" indicator if delta exceeds MOVING_DEG
 */
@Configurable
@TeleOp(name = "TEST Turret CR Servo Deadband", group = "Test")
public class TestTurretDeadband extends OpMode {

    private TurretSubsystemAbsoluteServos turret;

    // ===== Tune in Dashboard =====
    public static double STEP = 0.01;          // power step per button press
    public static double MAX_PWR = 0.30;       // don't go crazy; raise if needed
    public static double MOVING_DEG = 2.0;     // consider it "moving" if delta exceeds this
    public static double STICK_SCALE = 0.35;   // optional joystick control scale

    // command state
    private double cmdPower = 0.00;
    private boolean applyPower = false;
    private int sign = 1;

    // baseline angle for movement detection
    private double baselineDeg = 0.0;

    // edge tracking
    private boolean prevUp, prevDown, prevA, prevB, prevX, prevY;

    @Override
    public void init() {
        turret = new TurretSubsystemAbsoluteServos(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Important: make sure turret internal state is updated before reading angle
        turret.update();
        baselineDeg = turret.getCurrentAngleDeg();

        telemetry.addLine("Deadband test ready. Press START.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Ensure we're not in position mode; we want raw manual power test
        turret.setManualPower(0.0);
        applyPower = false;
        cmdPower = 0.0;

        turret.update();
        baselineDeg = turret.getCurrentAngleDeg();
    }

    @Override
    public void loop() {
        // Always update once per loop so angle changes are real
        turret.update();

        // ===== Button edges =====
        boolean up   = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean a    = gamepad1.a;
        boolean b    = gamepad1.b;
        boolean x    = gamepad1.x;
        boolean y    = gamepad1.y;

        boolean upEdge   = up && !prevUp;
        boolean downEdge = down && !prevDown;
        boolean aEdge    = a && !prevA;
        boolean bEdge    = b && !prevB;
        boolean xEdge    = x && !prevX;
        boolean yEdge    = y && !prevY;

        prevUp = up; prevDown = down; prevA = a; prevB = b; prevX = x; prevY = y;

        // ===== Optional joystick control (fine adjust) =====
        // If you want purely buttons, comment this block out.
        double stick = gamepad1.right_stick_x;
        if (Math.abs(stick) > 0.05) {
            cmdPower = Range.clip(stick * STICK_SCALE, -MAX_PWR, MAX_PWR);
            applyPower = true;
        }

        // ===== Step power with Dpad =====
        if (upEdge)   cmdPower += STEP;
        if (downEdge) cmdPower -= STEP;

        cmdPower = Range.clip(cmdPower, -MAX_PWR, MAX_PWR);

        // ===== Toggle applyPower =====
        if (aEdge) applyPower = !applyPower;

        // ===== Force stop (power=0) but keep applyPower on =====
        if (bEdge) {
            cmdPower = 0.0;
            applyPower = true;
        }

        // ===== Flip sign =====
        if (xEdge) sign *= -1;

        // ===== Reset baseline =====
        if (yEdge) {
            baselineDeg = turret.getCurrentAngleDeg();
        }

        // ===== Apply power =====
        double applied = applyPower ? (sign * cmdPower) : 0.0;
        turret.setManualPower(applied);

        // ===== Movement detection =====
        double curDeg = turret.getCurrentAngleDeg();
        double delta = curDeg - baselineDeg;
        boolean moving = Math.abs(delta) >= MOVING_DEG;

        // ===== Telemetry =====
        telemetry.addLine("Controls: DpadUp/Down step power | A toggle ON/OFF | B stop | X flip sign | Y set baseline");
        telemetry.addData("applyPower", applyPower);
        telemetry.addData("sign", sign);
        telemetry.addData("cmdPower", "%.3f", cmdPower);
        telemetry.addData("appliedPower", "%.3f", applied);

        telemetry.addData("angleDeg", "%.2f", curDeg);
        telemetry.addData("baselineDeg", "%.2f", baselineDeg);
        telemetry.addData("deltaDeg", "%.2f", delta);
        telemetry.addData("MOVING?", moving ? "YES" : "no");

        telemetry.addLine("Goal: find smallest |appliedPower| that makes MOVING?=YES consistently.");
        telemetry.addLine("Also set cmdPower=0 (B) and see if angle drifts (neutral offset).");

        telemetry.update();
    }
}

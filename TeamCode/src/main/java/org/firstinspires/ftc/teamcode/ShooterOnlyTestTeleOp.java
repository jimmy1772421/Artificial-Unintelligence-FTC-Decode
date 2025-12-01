package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Only TeleOp", group = "Test")
public class ShooterOnlyTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;

    // 0 = near field, 1 = far field (matches your ShooterSubsystem logic)
    private int fieldPos = 0;
    private boolean prevX = false;  // for toggling fieldPos on X press

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("ShooterOnlyTeleop ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = toggle shooter on/off");
        telemetry.addLine("  D-pad Up/Down = +250 / -250 RPM");
        telemetry.addLine("  X = toggle field position (near/far)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- Read buttons for shooter control ---
            boolean togglePressed   = gamepad1.a;
            boolean increasePressed = gamepad1.dpad_up;
            boolean decreasePressed = gamepad1.dpad_down;

            // --- Toggle field position with X (rising edge) ---
            boolean x = gamepad1.x;
            if (x && !prevX) {
                // Flip between 0 and 1
                fieldPos = (fieldPos == 0) ? 1 : 0;
            }
            prevX = x;

            // --- Update shooter subsystem ---
            shooter.update(togglePressed, increasePressed, decreasePressed, fieldPos);

            // --- Telemetry ---
            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Field Position", fieldPos == 0 ? "Near (0)" : "Far (1)");
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
        }

        // Make sure shooter is off when OpMode ends
        shooter.stop();
    }
}

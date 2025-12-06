package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Only TeleOp", group = "Test")
public class ShooterOnlyTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;

    // 0 = near field, 1 = far field
    private int fieldPos = 0;
    private boolean prevX = false;  // for toggling fieldPos on X press

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("ShooterOnlyTeleOp ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = toggle shooter on/off");
        telemetry.addLine("  D-pad Up/Down = +250 / -250 RPM (for current field)");
        telemetry.addLine("  X = toggle field position (near/far) + hood angle");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- Read buttons for shooter control ---
            boolean togglePressed   = gamepad1.a;        // on/off
            boolean increasePressed = gamepad1.dpad_up;  // +RPM for current field
            boolean decreasePressed = gamepad1.dpad_down;// -RPM for current field

            // --- Toggle field position with X (rising edge) ---
            boolean x = gamepad1.x;
            if (x && !prevX) {
                // Flip between 0 and 1
                fieldPos = (fieldPos == 0) ? 1 : 0;
            }
            prevX = x;

            // --- Update shooter subsystem (NOTE: int fieldPos, not boolean) ---
            shooter.update(
                    togglePressed,
                    increasePressed,
                    decreasePressed,
                    fieldPos     // 0 = near, 1 = far
            );

            // --- Telemetry ---
            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Field Position", fieldPos == 0 ? "Near (0)" : "Far (1)");
            telemetry.addData("Near RPM", "%.0f", shooter.getNearRpm());
            telemetry.addData("Far RPM", "%.0f", shooter.getFarRpm());
            telemetry.addData("Current Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Hood Pos", "%.2f", shooter.getHoodPosition());
            telemetry.update();
        }

        // Make sure shooter is off when OpMode ends
        shooter.stop();
    }
}

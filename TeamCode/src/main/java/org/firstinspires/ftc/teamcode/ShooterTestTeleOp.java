package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // A = toggle on/off
            // dpad_up = +250 RPM
            // dpad_down = -250 RPM
            shooter.update(
                    gamepad1.a,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down
            );

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem_NoEncoder;

@TeleOp(name = "Drive Test - No Encoder", group = "Test")
public class DriveTestNoEncoderTeleOp extends LinearOpMode {

    private DrivetrainSubsystem_NoEncoder drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init drivetrain
        drive = new DrivetrainSubsystem_NoEncoder(hardwareMap);

        telemetry.addLine("Drive Test (No Encoders) Ready");
        telemetry.addLine("Left stick: move / strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double leftX  = gamepad1.left_stick_x;
            double leftY  = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            // Call your subsystem method
            drive.driveNoEncoder(leftX, leftY, rightX);

            telemetry.addData("Drive Scale", drive.getDriveScale());
            telemetry.update();

            idle();
        }
    }
}

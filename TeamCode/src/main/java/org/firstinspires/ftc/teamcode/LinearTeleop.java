package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
//This test works
@TeleOp (name = "First_Teleop")
public class LinearTeleop extends LinearOpMode {
    private ShooterSubsystem shooter;
    private DrivetrainSubsystem drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        //intialization code
        //init hardware via subsystem
        shooter = new ShooterSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);


        //init Vars


        waitForStart();

        while(opModeIsActive()){
            drivetrain.drive(
                    gamepad1.left_stick_x,   // strafe
                    gamepad1.left_stick_y,   // forward/back
                    gamepad1.right_stick_x   // rotate
            );
            shooter.update(gamepad1.a);

            //Telemetry
            //shooter
            telemetry.addData("Shooter on", shooter.isOn());
            //drive
            telemetry.addData("Drive", "LX: %.2f LY: %.2f RX: %.2f",
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x);
            telemetry.update();
            telemetry.update();
        }
    }
}

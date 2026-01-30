package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestTeleop extends LinearOpMode {

    private double servoPosition = 0;  // start centered

    @Override
    public void runOpMode() throws InterruptedException {
        // initialization
        CRServo servoOne = hardwareMap.get(CRServo.class, "turretServoOne");
        CRServo servoTwo = hardwareMap.get(CRServo.class,"turretServoTwo");
        servoOne.setPower(servoPosition);
        servoTwo.setPower(servoPosition);

        waitForStart();

        while (opModeIsActive()) {

            // FTC gamepad: up is negative, down is positive
            // If you want up on the stick to move servo up, flip the sign as needed
            double stick = gamepad1.right_stick_y; // or -gamepad1.right_stick_y

            // Map stick [-1, 1] -> servo [0, 1], centered at 0.5
//            servoPosition = 0.5 + stick * 0.5;
            servoPosition =  stick;
            // Keep in valid range
//            servoPosition = Range.clip(servoPosition, 0.0, 1.0);

            // Apply to servo
            servoOne.setPower(servoPosition);
            servoTwo.setPower(servoPosition);

            // Telemetry
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Stick Y", stick);
            telemetry.update();

            // Let other stuff breathe
            idle();  // or sleep(10);
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem {
    //init motors
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private static final double MAX_TICKS_PER_SEC = 2815.0;
    public DrivetrainSubsystem (HardwareMap hardwareMap) {
        // --- HARDWARE MAPPING ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "BackRight");

        // --- MOTOR DIRECTION ---
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- MOTOR BEHAVIOR ---
        // Drivetrain and Climber set to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder setup
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive (double leftX, double leftY, double rightX){
        //Mecanum Drive using Swyft Drive V2
        double y = -leftY;       // forward/back
        double x = leftX;        // strafe
        double rx = rightX;      // rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        frontLeft.setVelocity(fl * MAX_TICKS_PER_SEC);
        backLeft.setVelocity(bl * MAX_TICKS_PER_SEC);
        frontRight.setVelocity(fr * MAX_TICKS_PER_SEC);
        backRight.setVelocity(br * MAX_TICKS_PER_SEC);

    }
}

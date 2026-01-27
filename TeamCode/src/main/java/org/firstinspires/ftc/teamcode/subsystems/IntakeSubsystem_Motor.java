package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem_Motor {

    // goBILDA 6000 RPM motor
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;
    private double speed = 5500.0;

    // Internal helpers

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }
    private static double inputToProportion(double input){return 0.5 + 0.5 * input;} // -1 - 1 -> 0-1

    private final DcMotorEx motor;
    private boolean isOn = false;

    public IntakeSubsystem_Motor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setSpeed(double input){ //0-1 input
        speed = input*PHYSICAL_MAX_RPM;
        motor.setVelocity(rpmToTicksPerSec(speed));
    }

    public void startIntake(){
        motor.setVelocity(rpmToTicksPerSec((speed)));
    }

    public void stopIntake(){
        motor.setVelocity(0);
    }

    public void reverseIntake(){
        speed = -speed;
    }


}

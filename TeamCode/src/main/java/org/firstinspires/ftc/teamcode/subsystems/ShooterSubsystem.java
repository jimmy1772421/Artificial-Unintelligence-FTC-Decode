package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private static final double MAX_TICKS_PER_SECOND = 2800.0; // 6000 RPM goBILDA motor

    private final DcMotorEx motorOne;
    private boolean isOn = false;
    private boolean lastButton = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {

        motorOne = hardwareMap.get(DcMotorEx.class,"motor_one");

        motorOne.setDirection(DcMotorSimple.Direction.FORWARD);

        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder ticks
    }

    public void update(boolean buttonPressed) {
        // Edge detection: button just went from not pressed -> pressed
        if (buttonPressed && !lastButton) {
            isOn = !isOn;
            motorOne.setVelocity(isOn ? MAX_TICKS_PER_SECOND : 0.0);
        }

        lastButton = buttonPressed;
    }

    public boolean isOn() {
        return isOn;
    }

    public void stop() {
        isOn = false;
        motorOne.setVelocity(0.0);
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoaderSubsystem {
    //initialization
    private final Servo servoOne;
    private final ElapsedTime cycleTimer = new ElapsedTime();
    //crop range
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.7;

    //set preset servo positions
    private static final double DEFAULT_POS = 0.0;
    private static final double UP_POS = 1.0;

    //set time in UP_POS
    private static final double UP_TIME_SEC = 0.10;

    private boolean isCycling = false;


    public LoaderSubsystem (HardwareMap hardwareMap) {
        servoOne = hardwareMap.get(Servo.class, "servo_one");
        //scale range
        servoOne.scaleRange(SERVO_MIN,SERVO_MAX);
        servoOne.setPosition(DEFAULT_POS);
    }

    public void setDefault(){
        servoOne.setPosition(DEFAULT_POS);
    }

    public void setUp(){
        servoOne.setPosition(UP_POS);
    }

    public void toggle(){
        if (servoOne.getPosition() < 0.5){
            servoOne.setPosition(UP_POS);
        }else{
            servoOne.setPosition(DEFAULT_POS);
        }
    }

    public void startCycle(){
        if (!isCycling){
            isCycling = true;
            servoOne.setPosition(DEFAULT_POS);
            isCycling = false;
        }
    }

    public void updateLoader(){
        if (isCycling){
            if (cycleTimer.seconds() >= UP_TIME_SEC){
                servoOne.setPosition(DEFAULT_POS);
                isCycling = false;
            }
        }
    }

    public boolean isCycling(){
        return isCycling;
    }
}

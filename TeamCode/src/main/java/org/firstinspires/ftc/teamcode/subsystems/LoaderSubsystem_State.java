package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoaderSubsystem_State {

    private final Servo servoOne;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double SERVO_MIN = 0.32;
    private static final double SERVO_MAX = 0.62;

    private static final double DEFAULT_POS = 0.0;
    private static final double UP_POS      = 1.0;

    private static final double UP_TIME_SEC = 0.10;

    private enum State { IDLE, UP_HOLD }
    private State state = State.IDLE;

    public LoaderSubsystem_State(HardwareMap hardwareMap) {
        servoOne = hardwareMap.get(Servo.class, "servo_one");
        servoOne.scaleRange(SERVO_MIN, SERVO_MAX);
        servoOne.setPosition(DEFAULT_POS);
    }

    public void startCycle() {
        if (state == State.IDLE) {
            servoOne.setPosition(UP_POS);
            timer.reset();
            state = State.UP_HOLD;
        }
    }

    public void updateLoader() {
        switch (state) {
            case IDLE:
                // do nothing
                break;

            case UP_HOLD:
                if (timer.seconds() >= UP_TIME_SEC) {
                    servoOne.setPosition(DEFAULT_POS);
                    state = State.IDLE;
                }
                break;
        }
    }

    public boolean isCycling() {
        return state != State.IDLE;
    }

    public void setDefault() { servoOne.setPosition(DEFAULT_POS); }
    public void setUp()      { servoOne.setPosition(UP_POS); }
}

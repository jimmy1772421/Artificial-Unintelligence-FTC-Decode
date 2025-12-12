package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightSubsystem {
    private final Servo shooterLight;
    private final double RED_COLOR = 0.277;
    private final double GREEN_COLOR = 0.5;
    private final double YELLOW_COLOR = 0.388;
    private final double OFF = 0;

    public LightSubsystem(HardwareMap hardwareMap) {
        shooterLight = hardwareMap.get(Servo.class, "shooterRGB");
        shooterLight.setPosition(OFF);

    }

    public void setColor(int color){
        if (color == 0){
            shooterLight.setPosition(0);
        } else if (color == 1) {
            shooterLight.setPosition(RED_COLOR);
        } else if (color == 2) {
            shooterLight.setPosition(YELLOW_COLOR);
        } else if (color == 3) {
            shooterLight.setPosition(GREEN_COLOR);

        }
    }
}

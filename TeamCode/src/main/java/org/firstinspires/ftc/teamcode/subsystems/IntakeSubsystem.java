package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
public class IntakeSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;

    private double intakeSpeed = 1.0;

    public IntakeSubsystem (HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void stopIntake(){
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
    }

    public void intakeSetPower(double gamepadInput){
        double intakePower = 0.5 + gamepadInput*0.5;
        leftServo.setPower(intakePower);
        rightServo.setPower(intakePower);

    }

    public void startIntake (){
        leftServo.setPower(intakeSpeed);
        rightServo.setPower(intakeSpeed);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(name = "Limelight Pattern Test", group = "Test")
public class LimelightPatternTest extends LinearOpMode {

    private VisionSubsystem vision;

    @Override
    public void runOpMode() {
        // Initialize Limelight / vision subsystem
        vision = new VisionSubsystem(hardwareMap);

        telemetry.addLine("Limelight Pattern Test Init");
        telemetry.addLine("Point LL3A at the AprilTags.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int pattern = vision.getPatternTag();   // 0, 21, 22, or 23

            telemetry.addData("Raw Pattern", pattern);

            // Human-readable description
            String desc;
            switch (pattern) {
                case 23:
                    desc = "23 = purple, purple, green";
                    break;
                case 22:
                    desc = "22 = purple, green, purple";
                    break;
                case 21:
                    desc = "21 = green, purple, purple";
                    break;
                case 0:
                default:
                    desc = "0 = no valid tag (shoot 0,1,2)";
                    break;
            }

            telemetry.addData("Pattern Meaning", desc);
            telemetry.update();

            // small delay so telemetry isn't absolutely flooded
            sleep(50);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spindexer Manual Tune", group = "Test")
public class SpindexerTuneTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Names must match your config
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        AnalogInput abs = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        telemetry.addLine("Spindexer Manual Tune");
        telemetry.addLine("D-pad LEFT/RIGHT: bump the spindexer a tiny amount");
        telemetry.addLine("Align SLOT 0 to the intake, then record RAW angle.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean lastLeft = false;
        boolean lastRight = false;

        while (opModeIsActive()) {
            boolean leftEdge  = gamepad1.dpad_left && !lastLeft;
            boolean rightEdge = gamepad1.dpad_right && !lastRight;
            lastLeft  = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            // Bump a tiny bit CCW
            if (leftEdge) {
                bump(motor, -0.2, 20); // power, ms; tune as needed
            }

            // Bump a tiny bit CW
            if (rightEdge) {
                bump(motor, 0.2, 20);
            }

            double v = abs.getVoltage();
            double rawDeg = (v / 3.3) * 360.0;

            telemetry.addData("Abs voltage", "%.3f V", v);
            telemetry.addData("Abs raw deg", "%.1f", rawDeg);
            telemetry.addLine("When slot 0 is PERFECTLY centered at intake,");
            telemetry.addLine("write down 'Abs raw deg' and use it as absMechOffsetDeg.");
            telemetry.update();
        }

        motor.setPower(0);
    }

    private void bump(DcMotorEx motor, double power, long ms) {
        try {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {}

        motor.setPower(power);
        sleep(ms);          // small, fixed-time move
        motor.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Measure Spindexer TPR", group = "Debug")
public class MeasureSpindexerTPR extends OpMode {

    private DcMotorEx spindexerMotor;

    private boolean measuring = false;
    private int startTicks = 0;
    private boolean prevA = false;

    @Override
    public void init() {
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");

        telemetry.addLine("TPR MEASUREMENT");
        telemetry.addLine("1) Press A -> records startTicks");
        telemetry.addLine("2) Rotate spindexer DISK by hand EXACTLY 1 full turn (360°)");
        telemetry.addLine("3) Press A again -> shows deltaTicks");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean aEdge = a && !prevA;
        prevA = a;

        if (aEdge) {
            if (!measuring) {
                startTicks = spindexerMotor.getCurrentPosition();
                measuring = true;
                telemetry.addData("StartTicks", startTicks);
                telemetry.addLine("Now rotate the disk exactly 360° by hand, then press A again.");
            } else {
                int current = spindexerMotor.getCurrentPosition();
                int delta = current - startTicks;
                telemetry.addData("StartTicks", startTicks);
                telemetry.addData("EndTicks", current);
                telemetry.addData("DeltaTicks (TPR)", delta);
                telemetry.addLine("Use abs(delta) as TICKS_PER_REV for the DISK.");
                measuring = false;
            }
        }

        telemetry.update();
    }
}

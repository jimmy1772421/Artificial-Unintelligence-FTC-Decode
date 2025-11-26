package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.graphics.Color;

@TeleOp(name = "Color HSV Test", group = "Test")
public class ColorHsvTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Make sure this name matches your RC config
        RevColorSensorV3 sensor =
                hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");

        telemetry.addLine("Color HSV Test");
        telemetry.addLine("Place a GREEN or PURPLE pixel under the sensor.");
        telemetry.addLine("Then press Play and watch the HSV values.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            // Raw RGB from the sensor
            int r = sensor.red();
            int g = sensor.green();
            int b = sensor.blue();

            // Convert to HSV using Android's Color utility
            int rgb = Color.rgb(r, g, b);   // packs r,g,b into one int
            Color.colorToHSV(rgb, hsv);     // fills hsv[0]=H, hsv[1]=S, hsv[2]=V

            float hue = hsv[0];   // 0..360
            float sat = hsv[1];   // 0..1
            float val = hsv[2];   // 0..1

            telemetry.addData("R,G,B", "%d, %d, %d", r, g, b);
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Sat", "%.2f", sat);
            telemetry.addData("Val", "%.2f", val);
            telemetry.update();
        }
    }
}

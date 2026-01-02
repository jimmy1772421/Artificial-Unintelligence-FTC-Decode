package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Configurable
@TeleOp(name = "TUNE Turret (Panels)", group = "Tuning")
public class TurretTunerPanels extends OpMode {

    // ======================
    // Tunables (Panels)
    // ======================
    public static double targetDeg = 0.0;      // commanded turret angle
    public static double stepDeg = 5.0;        // dpad L/R adjusts target by this
    public static double manualScale = 0.5;    // stick power scaling
    public static double settleTolDeg = 3.0;   // "at target" threshold

    public static boolean holdWhenIdle = true; // if false, stick idle -> motor off (manual 0)

    // Vision tracking (camera mounted ON the turret)
    public static boolean trackEnabled = false;
    public static int trackTagId = 24;
    public static double txDeadbandDeg = 0.5;
    public static double txMaxStepDeg = 2.0;   // max degrees to change setpoint per loop
    public static double txSign = 1.0;         // flip to -1 if it turns the wrong way

    // ======================
    // Hardware
    // ======================
    private TurretSubsystem turret;
    private VisionSubsystem vision;

    private TelemetryManager telemetryM;

    // state
    private double lastCommandedTarget = Double.NaN;

    // edges
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevDpadL, prevDpadR, prevDpadU, prevDpadD;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Turret Tuner ready");
        telemetry.addLine("Right stick = manual");
        telemetry.addLine("Dpad L/R = target +/- step");
        telemetry.addLine("A=0deg, Y=hold current, X=toggle tracking, B=toggle holdWhenIdle");
        telemetry.update();
    }

    @Override
    public void start() {
        // start by holding current angle
        targetDeg = turret.getCurrentAngleDeg();
        lastCommandedTarget = Double.NaN; // force first command
    }

    @Override
    public void loop() {
        // ----------------------
        // Edge buttons
        // ----------------------
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean aEdge = a && !prevA;
        boolean bEdge = b && !prevB;
        boolean xEdge = x && !prevX;
        boolean yEdge = y && !prevY;

        prevA = a; prevB = b; prevX = x; prevY = y;

        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;
        boolean du = gamepad1.dpad_up;
        boolean dd = gamepad1.dpad_down;

        boolean dlEdge = dl && !prevDpadL;
        boolean drEdge = dr && !prevDpadR;
        boolean duEdge = du && !prevDpadU;
        boolean ddEdge = dd && !prevDpadD;

        prevDpadL = dl; prevDpadR = dr; prevDpadU = du; prevDpadD = dd;

        if (xEdge) trackEnabled = !trackEnabled;
        if (bEdge) holdWhenIdle = !holdWhenIdle;

        if (aEdge) targetDeg = 0.0; // home/center command
        if (yEdge) targetDeg = turret.getCurrentAngleDeg(); // hold current

        if (dlEdge) targetDeg -= stepDeg;
        if (drEdge) targetDeg += stepDeg;

        // optional: adjust step size
        if (duEdge) stepDeg = Math.min(stepDeg + 1.0, 45.0);
        if (ddEdge) stepDeg = Math.max(stepDeg - 1.0, 0.5);

        // ----------------------
        // Manual vs tracking vs hold/off
        // ----------------------
        double stickX = gamepad1.right_stick_x;
        boolean manualActive = Math.abs(stickX) > 0.05;

        double tx = Double.NaN;
        String seen = "";

        if (manualActive) {
            // Manual overrides tracking
            turret.setManualPower(stickX * manualScale);
        } else if (trackEnabled) {
            // Camera-on-turret: tx is basically the angular error (deg)
            tx = vision.getTagTxDegOrNaN(trackTagId);
            seen = vision.getSeenTagIdsString();

            if (!Double.isNaN(tx) && Math.abs(tx) > txDeadbandDeg) {
                double step = Range.clip(txSign * tx, -txMaxStepDeg, txMaxStepDeg);
                targetDeg = turret.getCurrentAngleDeg() + step;
            } else {
                // no tag / tiny tx: hold current angle
                targetDeg = turret.getCurrentAngleDeg();
            }

            commandTargetIfChanged(targetDeg);

        } else if (holdWhenIdle) {
            commandTargetIfChanged(targetDeg);
        } else {
            turret.setManualPower(0.0);
        }

        turret.update();

        // ----------------------
        // Telemetry
        // ----------------------
        double cur = turret.getCurrentAngleDeg();
        double tgt = turret.getTargetAngleDeg();
        double err = tgt - cur;

        telemetry.addData("Mode",
                manualActive ? "MANUAL" : (trackEnabled ? "TRACK" : (holdWhenIdle ? "HOLD" : "OFF")));
        telemetry.addData("TargetDeg(cmd)", "%.1f", targetDeg);
        telemetry.addData("TargetDeg(subsys)", "%.1f", tgt);
        telemetry.addData("CurrentDeg", "%.1f", cur);
        telemetry.addData("ErrorDeg", "%.1f", err);
        telemetry.addData("AtTarget", Math.abs(err) < settleTolDeg);

        telemetry.addData("TrackEnabled", trackEnabled);
        telemetry.addData("TrackTagId", trackTagId);
        telemetry.addData("txDeg", Double.isNaN(tx) ? "NaN" : String.format("%.2f", tx));
        telemetry.addData("SeenTags", seen);

        telemetryM.debug("turret_deg", cur);
        telemetryM.debug("turret_target", tgt);
        telemetryM.debug("turret_err", err);
        telemetryM.debug("track_enabled", trackEnabled);

        telemetry.update();
        telemetryM.update();
    }

    private void commandTargetIfChanged(double target) {
        if (Double.isNaN(lastCommandedTarget) || Math.abs(target - lastCommandedTarget) > 0.05) {
            turret.goToAngle(target);
            lastCommandedTarget = target;
        }
    }
}

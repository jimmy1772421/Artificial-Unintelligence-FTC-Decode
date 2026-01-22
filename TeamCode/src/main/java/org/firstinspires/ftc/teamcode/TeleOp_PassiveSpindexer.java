package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_Passive_State_new_Incremental;

@Configurable
@TeleOp(name = "TEST Passive Spindexer + Shooter", group = "Test")
public class TeleOp_PassiveSpindexer extends OpMode {

    // ===== Drive =====
    private Follower follower;
    private boolean slowMode = false;
    private boolean prevSlow = false;

    // ===== Subsystems =====
    private ShooterSubsystemFF shooter;
    private SpindexerSubsystem_Passive_State_new_Incremental spindexer;
    private LoaderSubsystem loader;

    // ===== Telemetry =====
    private TelemetryManager telemetryM;
    private FtcDashboard dashboard;

    // ===== Shooter controls =====
    private boolean shooterOn = false;
    private boolean prevShooterToggle = false;

    private int fieldPos = 0; // 0=near, 1=far
    private boolean prevFieldToggle = false;

    // ===== Spindexer controls =====
    private boolean prevShoot = false;
    private boolean prevRehome = false;
    private boolean prevReverse = false;
    private boolean prevGoPastUp = false;
    private boolean prevGoPastDown = false;

    // ===== Pattern select =====
    private int driverPatternTag = 0;
    private boolean prev2Up = false, prev2Right = false, prev2Left = false, prev2Down = false;

    private int lastSpTicks = 0;

    // ===== Debug stats for encoder sign vs motor power sign =====
    private int signMatch = 0;
    private int signMismatch = 0;
    private int signNA = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        if (PoseStorage.lastPose != null) {
            follower.setStartingPose(PoseStorage.lastPose);
        }
        follower.update();

        // Panels
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Dashboard telemetry mirror
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Subsystems
        shooter   = new ShooterSubsystemFF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem_Passive_State_new_Incremental(hardwareMap);

        // Default tunables you can adjust live
        // NOTE: staging is defined as PRESHOOT + GO_PAST in your new subsystem
        SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG = 60.0; // start here

        telemetry.addLine("TEST Passive Spindexer + Shooter ready.");
        telemetry.addLine("gamepad1.y=shoot | x=shooter toggle | rb/lb=gopast | back=reverse");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // For a TEST opmode, it’s usually safest to reset to intake.
        // If you want Auto->TeleOp continuity, comment this out.
        spindexer.homeToIntake();
        lastSpTicks = spindexer.getEncoder();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // ===== Drive (gamepad2) =====
        follower.update();

        boolean slow = gamepad2.a;
        if (slow && !prevSlow) slowMode = !slowMode;
        prevSlow = slow;

        double driveScale = slowMode ? 0.4 : 1.0;

        double leftX  = gamepad2.left_stick_x;
        double leftY  = gamepad2.left_stick_y;
        double rightX = gamepad2.right_stick_x;

        follower.setTeleOpDrive(
                -leftY * driveScale,
                -leftX * driveScale,
                -rightX * driveScale,
                true
        );

        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        // ===== Shooter toggle (gamepad1.x) =====
        boolean shooterToggle = gamepad1.x;
        if (shooterToggle && !prevShooterToggle) shooterOn = !shooterOn;
        prevShooterToggle = shooterToggle;

        // field pos toggle (gamepad1 left stick button)
        boolean fieldToggle = gamepad1.left_stick_button;
        if (fieldToggle && !prevFieldToggle) fieldPos = (fieldPos == 0) ? 1 : 0;
        prevFieldToggle = fieldToggle;

        // ===== Spindexer shoot edge (gamepad1.y) =====
        boolean shootBtn = gamepad1.y;
        boolean yEdge = shootBtn && !prevShoot;
        prevShoot = shootBtn;

        // rehome to intake (gamepad1 right stick button)
        boolean rehome = gamepad1.right_stick_button;
        if (rehome && !prevRehome) spindexer.homeToIntake();
        prevRehome = rehome;

        // reverse motor toggle (gamepad1.back)
        boolean rev = gamepad1.back;
        if (rev && !prevReverse) {
            SpindexerSubsystem_Passive_State_new_Incremental.REVERSE_MOTOR =
                    !SpindexerSubsystem_Passive_State_new_Incremental.REVERSE_MOTOR;
        }
        prevReverse = rev;

        // GO_PAST live tune (gamepad1 rb/lb)
        boolean up = gamepad1.right_bumper;
        boolean dn = gamepad1.left_bumper;

        if (up && !prevGoPastUp) SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG += 5.0;
        if (dn && !prevGoPastDown) SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG -= 5.0;

        prevGoPastUp = up;
        prevGoPastDown = dn;

        // Clamp goPast a bit so it doesn’t go crazy
        if (SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG < 0) {
            SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG = 0;
        }
        if (SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG > 180) {
            SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG = 180;
        }

        // ===== Pattern select (gamepad2 dpad) =====
        boolean dUp2    = gamepad2.dpad_up;
        boolean dRight2 = gamepad2.dpad_right;
        boolean dLeft2  = gamepad2.dpad_left;
        boolean dDown2  = gamepad2.dpad_down;

        if (dUp2 && !prev2Up) driverPatternTag = 23;
        if (dRight2 && !prev2Right) driverPatternTag = 22;
        if (dLeft2 && !prev2Left) driverPatternTag = 21;
        if (dDown2 && !prev2Down) driverPatternTag = 0;

        prev2Up = dUp2;
        prev2Right = dRight2;
        prev2Left = dLeft2;
        prev2Down = dDown2;

        // ===== Spindexer update =====
        // loader is passed for signature compatibility (passive mode ignores it)
        boolean spFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

        // Keep loader updated if your subsystem needs it elsewhere
        loader.updateLoader();

        // ===== Shooter update =====
        // IMPORTANT: shooter.update expects dpad_up/down inputs each loop; we pass them directly.
        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );

        // ===== Telemetry =====
        SpindexerSubsystem_Passive_State_new_Incremental.Ball[] slots = spindexer.getSlots();

        telemetry.addData("ShooterOn", shooterOn);
        telemetry.addData("FieldPos", fieldPos == 0 ? "NEAR" : "FAR");
        telemetry.addData("TargetRPM", "%.0f", shooter.getTargetRpm());
        telemetry.addData("RPM(est)", "%.0f", shooter.getCurrentRpmEstimate());

        telemetry.addData("Sp Full", spFull);
        telemetry.addData("Sp Ejecting", spindexer.isEjecting());
        telemetry.addData("Sp Angle", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("Sp GO_PAST", "%.1f", SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG);
        telemetry.addData("Sp Reverse", SpindexerSubsystem_Passive_State_new_Incremental.REVERSE_MOTOR);
        telemetry.addData("Sp Tag", driverPatternTag);
        telemetry.addData("Sp slots", "%s %s %s", slots[0], slots[1], slots[2]);

        telemetryM.addData("drive/slow", slowMode);
        telemetryM.addData("sp/angle", spindexer.getCurrentAngleDeg());
        telemetryM.addData("sp/gopast", SpindexerSubsystem_Passive_State_new_Incremental.GO_PAST_ANGLE_DEG);
        telemetryM.addData("shooter/on", shooterOn);
        telemetryM.addData("shooter/rpm", shooter.getCurrentRpmEstimate());
        int t = spindexer.getEncoder();
        int dt = t - lastSpTicks;
        lastSpTicks = t;

        telemetry.addData("SpTicks", t);
        telemetry.addData("dTicks", dt);
        telemetry.addData("SpAngle", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("SpErrToTarget", "%.1f", spindexer.getAngleErrorToTargetDeg());
        double pReq = spindexer.dbgLastRequestedPower();
        double pApp = spindexer.dbgLastAppliedPower();

// Signs: +1 / 0 / -1
        int tickSign = Integer.compare(dt, 0);
        int pwrSign  = Double.compare(pApp, 0.0);

        String tickS = tickSign > 0 ? "+" : (tickSign < 0 ? "-" : "0");
        String pwrS  = pwrSign  > 0 ? "+" : (pwrSign  < 0 ? "-" : "0");

        boolean meaningful = (tickSign != 0) && (pwrSign != 0);
        boolean match = meaningful && (tickSign == pwrSign);

        if (!meaningful) signNA++;
        else if (match) signMatch++;
        else signMismatch++;

// Telemetry
        telemetry.addData("Dbg pwrReq", "%.2f", pReq);
        telemetry.addData("Dbg pwrApp", "%.2f", pApp);
        telemetry.addData("Dbg sign (pwr/ticks)", "%s / %s  => %s", pwrS, tickS, meaningful ? (match ? "MATCH" : "MISMATCH") : "N/A");
        telemetry.addData("Dbg sign stats", "match=%d mismatch=%d na=%d", signMatch, signMismatch, signNA);

        telemetryM.update(telemetry);
        telemetry.update();
    }
}

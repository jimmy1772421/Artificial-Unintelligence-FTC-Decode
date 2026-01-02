package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp with pedro and pidf", group = "Test")
public class TeleOp_pedro_pidf extends OpMode {

    // ===== PEDRO FOLLOWER / DRIVE =====
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;

    // ===== SUBSYSTEMS =====
    private ShooterSubsystemPIDF shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem_State_new spindexer;
    private LoaderSubsystem loader;
    private TurretSubsystem turret;

    // ===== DASHBOARD =====
    private FtcDashboard dashboard;

    // ===== STATE VARIABLES =====
    private boolean prev2a = false;

    private int fieldPos = 0;
    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // turret buttons (gamepad1)
    private boolean prevA, prevDpadL, prevDpadR;
    private boolean turretManualOverride = false;

    boolean readyForIntake = true;
    private long shooterSpinDownDeadline = 0;
    private boolean lastEjecting = false;

    private int driverPatternTag = 0;
    private boolean prev2Up, prev2Right, prev2Left, prev2Down;

    private boolean prevLeftStick = false;
    private boolean prevRehome = false;

    private boolean prev2LeftBumper = false;
    private boolean prev2RightBumper = false;

    // Edge detection for gamepad1
    private boolean prevStart = false;
    private boolean prevY = false;
    private boolean prevB1 = false;

    // ===== SHOOT POSES (tune these numbers) =====
    public static Pose SHOOT_POSE_NEAR = new Pose(87, 87, Math.toRadians(45));
    public static Pose SHOOT_POSE_FAR  = new Pose(84, 16, Math.toRadians(90));

    // ===== SHOOT ASSIST STATE =====
    private boolean shootAssistActive = false;
    private boolean prevShootAssistBtn = false;
    private Pose activeShootPose = SHOOT_POSE_NEAR;

    private boolean autoFirePulse = false;
    private long autoFireCooldownUntil = 0;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Init Panels field offsets for Pedro
        Drawing.init();

        // Use TeleOp-specific startingPose if set; otherwise use Drawing's configurable pose
        Pose startPose = (startingPose != null)
                ? startingPose
                : Drawing.getStartingPose();

        follower.setStartingPose(startPose);
        follower.update();


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(45),
                                0.8
                        )
                )
                .build();

        shooter   = new ShooterSubsystemPIDF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem_State_new(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("TeleOp with Pedro + Shooter PIDF");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        spindexer.homeToIntake();
        shooterSpinDownDeadline = 0;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // ===== UPDATE PEDRO FOLLOWER =====
        follower.update();
        telemetryM.update();

        // Draw robot pose + heading on Panels field
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();


        // ===== DRIVETRAIN =====
        double leftX  = gamepad2.left_stick_x;
        double leftY  = gamepad2.left_stick_y;
        double rightX = gamepad2.right_stick_x;

        boolean a2 = gamepad2.a;
        if (a2 && !prev2a) slowMode = !slowMode;
        prev2a = a2;

        double driveScale = slowMode ? 0.4 : 1.0;

        if (!automatedDrive) {
            // Typical convention: forward is -leftY, strafe is -leftX, turn is -rightX
            follower.setTeleOpDrive(
                    -leftY * driveScale,
                    -leftX * driveScale,
                    -rightX * driveScale,
                    true
            );
        }

        // ===== SHOOT ASSIST (fieldPos: 0 near, 1 far) =====
        boolean shootAssistBtn = gamepad2.y;
        boolean shootAssistEdge = shootAssistBtn && !prevShootAssistBtn;
        prevShootAssistBtn = shootAssistBtn;

        boolean b1Edge = gamepad1.b && !prevB1;
        prevB1 = gamepad1.b;

        boolean driverOverride =
                Math.abs(gamepad2.left_stick_x) > 0.2 ||
                        Math.abs(gamepad2.left_stick_y) > 0.2 ||
                        Math.abs(gamepad2.right_stick_x) > 0.2;

        if (shootAssistEdge) {
            if (!shootAssistActive) {
                Pose target = (fieldPos == 0) ? SHOOT_POSE_NEAR : SHOOT_POSE_FAR;
                startShootAssist(target);
            } else {
                cancelShootAssist();
            }
        }

        if (shootAssistActive && (b1Edge || driverOverride)) {
            cancelShootAssist();
        }

// If the path finished, give control back (but keep shoot mode active until you cancel)
        if (shootAssistActive && automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // ===== Y edge (user OR auto-fire pulse) =====
        boolean yUser = gamepad1.y;
        boolean yEdgeUser = yUser && !prevY;
        prevY = yUser;

        boolean yEdge = yEdgeUser || autoFirePulse;
        autoFirePulse = false; // consume pulse


        // ===== SHOOTER FIELD POSITION TOGGLE =====
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !prevLeftStick) fieldPos = (fieldPos == 0) ? 1 : 0;
        prevLeftStick = leftStickButton;

        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );

        // ===== LOADER =====
        loader.updateLoader();

        // ===== INTAKE =====
        if (intakeOn) intake.startIntake();
        else intake.stopIntake();

        // ===== TURRET CONTROL =====
        boolean a  = gamepad1.a;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        if (a && !prevA) turret.goToAngle(0.0);
        if (dl && !prevDpadL) turret.goToAngle(90.0);
        if (dr && !prevDpadR) turret.goToAngle(-90.0);

        prevA = a;
        prevDpadL = dl;
        prevDpadR = dr;

        // Manual turret stick (does NOT cancel auto unless you actually move the stick)
        double stickX = gamepad1.right_stick_x;
        boolean manualActive = Math.abs(stickX) > 0.05;

        if (manualActive) {
            turret.setManualPower(stickX * 0.5);
            turretManualOverride = true;
        } else {
            // If we were manual last loop, explicitly stop the motor
            if (turretManualOverride) {
                turret.setManualPower(0.0);
                turretManualOverride = false;
            }
        }

        turret.update();

        // ===== SPINDEXER STEP (state machine + PIDF + hold) =====
        //spindexer.periodic();

        // Rehome spindexer
        boolean rehomeButton = gamepad1.right_stick_button;
        if (rehomeButton && !prevRehome) spindexer.homeToIntake();
        prevRehome = rehomeButton;

        // driver2 sets pattern
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



        // State spindexer main update
        spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

        // --- Eject / shooter timing ---


        boolean ejecting   = spindexer.isEjecting();
        boolean hasAnyBall = spindexer.hasAnyBall();

        if (lastEjecting && !ejecting && !hasAnyBall) {
            shooterSpinDownDeadline = now + 2000;
        }
        lastEjecting = ejecting;

        boolean wantShooter;
        boolean wantIntake;

        if (spindexerIsFull) {
            wantShooter = true;
            wantIntake  = false;
        } else if (ejecting || now < shooterSpinDownDeadline) {
            wantShooter = true;
            wantIntake  = false;
        } else {
            wantShooter = false;
            wantIntake  = true;
        }

        shooterOn = wantShooter;
        intakeOn  = true;

        // ===== FORCE-REGISTER (driver 2 bumpers) =====
        boolean lb2 = gamepad2.left_bumper;
        boolean rb2 = gamepad2.right_bumper;

        if (lb2 && !prev2LeftBumper) spindexer.forceIntakeSlotGreen(telemetry);
        if (rb2 && !prev2RightBumper) spindexer.forceIntakeSlotPurple(telemetry);

        prev2LeftBumper = lb2;
        prev2RightBumper = rb2;

        if (shootAssistActive) {
            boolean arrived = atPose(follower.getPose(), activeShootPose, 2.0, 6.0);

            double target = shooter.getTargetRpm();
            double current = shooter.getCurrentRpmEstimate();
            boolean rpmReady = Math.abs(target - current) < 150;

            if (arrived && rpmReady && now > autoFireCooldownUntil) {
                autoFirePulse = true;          // fires NEXT loop via yEdge OR
                autoFireCooldownUntil = now + 800;
            }
        }



        // ===== TELEMETRY =====
        SpindexerSubsystem_State_new.Ball[] s = spindexer.getSlots();

        readyForIntake = !spindexer.isEjecting() && !spindexer.isAutoRotating();

        telemetry.addData("Spd intakeSlot", spindexer.getIntakeSlotIndex());
        telemetry.addData("Spd readyForIntake", readyForIntake);
        telemetry.addData("Spd full", spindexer.isFull());
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("Spd slot[0]", s[0]);
        telemetry.addData("Spd slot[1]", s[1]);
        telemetry.addData("Spd slot[2]", s[2]);
        telemetry.addData("Spd angle(enc)", "%.1f", spindexer.getCurrentAngleDeg());
        spindexer.debugAbsAngle(telemetry);

        double target = shooter.getTargetRpm();
        double current = shooter.getCurrentRpmEstimate();
        double error = target - current;

        telemetry.addData("Shooter On", shooter.isOn());
        telemetry.addData("Field Pos", (fieldPos == 0) ? "NEAR" : "FAR");
        telemetry.addData("Target RPM", "%.0f", target);
        telemetry.addData("Current RPM (est)", "%.0f", current);
        telemetry.addData("RPM Error", "%.0f", error);

        telemetry.addData("Pattern Tag", driverPatternTag);
        telemetry.addData("Pattern Order", spindexer.getGamePattern());

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rpm", current);
        packet.put("target", target);
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    private PathChain buildLineToPose(Pose target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, target))) // straight line
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                target.getHeading(),
                                0.8
                        )
                )
                .build();
    }

    private void startShootAssist(Pose target) {
        activeShootPose = target;
        follower.followPath(buildLineToPose(target));
        automatedDrive = true;
        shootAssistActive = true;
    }

    private void cancelShootAssist() {
        follower.startTeleopDrive();
        automatedDrive = false;
        shootAssistActive = false;
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    private boolean atPose(Pose cur, Pose target, double posTolIn, double headingTolDeg) {
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        double dist = Math.hypot(dx, dy);

        double dh = Math.toDegrees(Math.abs(wrapRad(cur.getHeading() - target.getHeading())));
        return dist <= posTolIn && dh <= headingTolDeg;
    }

}

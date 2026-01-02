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
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

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
    private VisionSubsystem vision;

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
    public static Pose SHOOT_POSE_FAR  = new Pose(84, 16, Math.toRadians(65));

    // ===== SHOOT ASSIST STATE =====
    private boolean shootAssistActive = false;
    private boolean prevShootAssistBtn = false;
    private Pose activeShootPose = SHOOT_POSE_NEAR;

    private boolean autoFirePulse = false;
    private long autoFireCooldownUntil = 0;

    // ===== TURRET TRACK TOGGLE =====
    private boolean turretTrackEnabled = false;
    private boolean prevTurretTrackToggle = false;

    // ===== TURRET OWNER STATE =====
    private boolean turretPositionCommandActive = false; // true after goToAngle until close enough

    // Tunables
    public static int TRACK_TAG_ID = 24;
    // ===== Vision->Turret angle tracking (camera on turret) =====
    public static double TX_DEADBAND_DEG = 0.5;     // ignore tiny tx noise
    public static double TX_MAX_STEP_DEG = 2.0;     // max deg to change target per loop (smooth)
    public static double TX_SIGN = 1.0;             // flip to -1.0 if it turns the wrong way
    public static double TURRET_ANGLE_TOL_DEG = 3.0;

    public static double TRACK_TX_DEADBAND = 0.5;  // deg

    // ===== POST-PATH ACTIONS (auto home + relocalize once) =====
    private boolean postPathActionsDone = false;
    private long postPathActionsTimeMs = 0;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Init Panels field offsets for Pedro
        Drawing.init();

        // Use Auto's passed on starting pose or TeleOp-specific startingPose if set; otherwise use Drawing's configurable pose
        Pose startPose =
                (PoseStorage.lastPose != null) ? PoseStorage.lastPose :
                        (startingPose != null) ? startingPose :
                                Drawing.getStartingPose();

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
        vision    = new VisionSubsystem(hardwareMap);


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

        if (shootAssistActive && automatedDrive && !follower.isBusy()) {

            // give drive control back
            follower.startTeleopDrive();
            automatedDrive = false;

            // run these only once per shoot-assist run
            if (!postPathActionsDone) {
                // 1) Home turret to 0 deg
                turret.goToAngle(0.0);
                turretPositionCommandActive = true;

                // 2) Relocalize using Limelight (MegaTag2)
                Pose visionPose = vision.getPedroPoseFromLimelight(0, follower); // 0 = accept any tag
                if (visionPose != null) {
                    // Pedro-safe method you already have:
                    follower.setStartingPose(visionPose);
                    follower.update();
                }

                postPathActionsDone = true;
                postPathActionsTimeMs = now;
            }
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

        // ===== TOGGLE TURRET TRACK MODE (gamepad1.x) =====
        boolean toggleTrack = gamepad1.x;
        if (toggleTrack && !prevTurretTrackToggle) {
            turretTrackEnabled = !turretTrackEnabled;
        }
        prevTurretTrackToggle = toggleTrack;

// ===== PRESET ANGLES (edge) =====
        boolean a  = gamepad1.a;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        boolean aEdge  = a  && !prevA;
        boolean dlEdge = dl && !prevDpadL;
        boolean drEdge = dr && !prevDpadR;

        prevA = a;
        prevDpadL = dl;
        prevDpadR = dr;

        if (aEdge)  { turret.goToAngle(0.0);   turretPositionCommandActive = true; }
        if (dlEdge) { turret.goToAngle(90.0);  turretPositionCommandActive = true; }
        if (drEdge) { turret.goToAngle(-90.0); turretPositionCommandActive = true; }

// ===== MANUAL vs TRACKING vs HOLD =====
        double stickX = gamepad1.right_stick_x;
        boolean manualActive = Math.abs(stickX) > 0.05;

        if (manualActive) {
            // Manual wins; also disable tracking while driver is actively steering turret
            turretTrackEnabled = false;
            turretPositionCommandActive = false;

            turret.setManualPower(stickX * 0.5);
            turretManualOverride = true;

        } else if (turretTrackEnabled) {
            // Camera is on turret => tx is basically "degrees you need to turn"
            double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);

            // If we see tag and tx is meaningful, walk the target angle toward it
            if (!Double.isNaN(tx) && Math.abs(tx) > TX_DEADBAND_DEG) {
                double step = Range.clip(TX_SIGN * tx, -TX_MAX_STEP_DEG, TX_MAX_STEP_DEG);
                double newTarget = turret.getCurrentAngleDeg() + step;
                turret.goToAngle(newTarget);
                turretPositionCommandActive = true; // we are actively commanding angles
            } else {
                // no tag / tiny tx -> just hold where you are
                turret.goToAngle(turret.getCurrentAngleDeg());
                turretPositionCommandActive = true;
            }

            turretManualOverride = false;

        } else if (turretPositionCommandActive) {
            // Let RUN_TO_POSITION finish
            double err = turret.getTargetAngleDeg() - turret.getCurrentAngleDeg();
            if (Math.abs(err) < TURRET_ANGLE_TOL_DEG) {
                turretPositionCommandActive = false;
            }

        } else {
            // Default: stick idle + not tracking => stop motor (your old behavior)
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
            boolean turretAtZero = Math.abs(turret.getCurrentAngleDeg()) < TURRET_ANGLE_TOL_DEG;

            boolean arrived = atPose(follower.getPose(), activeShootPose, 2.0, 6.0);

            double target = shooter.getTargetRpm();
            double current = shooter.getCurrentRpmEstimate();
            boolean rpmReady = Math.abs(target - current) < 150;

            // require postPathActionsDone so we actually homed+relocalized first
            if (postPathActionsDone && turretAtZero && arrived && rpmReady && now > autoFireCooldownUntil) {
                autoFirePulse = true;
                autoFireCooldownUntil = now + 800;
            }

            if (postPathActionsDone && turretAtZero) {
                turretTrackEnabled = true;
            }
        }

        //record pose
        PoseStorage.lastPose = follower.getPose();

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
        telemetry.addData("Turret Track", turretTrackEnabled);
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("Tag24 tx", "%.2f", vision.getTagTxDegOrNaN(24));


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

        postPathActionsDone = false; // reset for this run
    }


    private void cancelShootAssist() {
        follower.startTeleopDrive();
        automatedDrive = false;
        shootAssistActive = false;

        postPathActionsDone = false;
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

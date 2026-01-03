package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

@Autonomous(name = "RedAutoUp", group = "Examples")
public class RedAutoUp extends OpMode {

    // ============================
    // ===== TURRET TRACKING ======
    // ============================
    public static boolean ENABLE_TURRET_TRACKING = true;

    public static int TRACK_TAG_ID = 24;
    public static double TRACK_KP = 0.02;           // power per deg of tx
    public static double TRACK_MAX_POWER = 0.25;    // clamp
    public static double TRACK_TX_DEADBAND = 0.4;   // deg
    public static double TRACK_SIGN = 1.0;          // set to -1.0 if turret turns the wrong way

    // ============================
    // ===== PEDRO / PANELS =======
    // ============================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private FieldManager panelsField;
    private static final double ROBOT_RADIUS = 9;

    // ============================
    // ===== SUBSYSTEMS ===========
    // ============================
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private ShooterSubsystemPIDF shooter;
    private LoaderSubsystem loader;
    private SpindexerSubsystem_State_new spindexer;
    // ============================
    // ===== POSES / PATHS ========
    // ============================
    private final Pose startPose   = new Pose(109, 134, Math.toRadians(90));
    private final Pose scorePose   = new Pose(80, 80, Math.toRadians(45));
    private final Pose prePickup1Pose = new Pose(99, 83.5, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(124, 83.5, Math.toRadians(0));
    private final Pose prePickup2Pose = new Pose(99, 59, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(124, 59, Math.toRadians(0));
    private final Pose prePickup3Pose = new Pose(99, 35, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(124, 35, Math.toRadians(0));

    private Path scorePreload;
    private PathChain goPrePickup1, creepToPickup1, scorePickup1, goPrePickup2, creepToPickup2,
            scorePickup2, goPrePickup3, creepToPickup3, scorePickup3;

    //Power in path
    private static final double PWR_FAST  = 0.85;
    private static final double PWR_CREEP = 0.35;


    // ===== TURRET PRE-SHOOT HOLD =====
    public static boolean HOLD_TURRET_90_UNTIL_SCOREPOSE = true;

    // If true: turret faces a FIELD heading (deg). If false: turret just goes to +90 deg turret angle.
    public static boolean HOLD_FIELD_CENTRIC = true;

    // the field direction you want the turret to face before first shot
    public static double HOLD_FIELD_DEG = 90.0;

    // if not field-centric, turret angle to hold
    public static double HOLD_TURRET_DEG = 90.0;

    // how close to scorePose counts as "arrived"
    public static double SCOREPOSE_POS_TOL_IN = 2.0;
    public static double SCOREPOSE_HEAD_TOL_DEG = 8.0;

    private int shootState = 0;
    private final Timer shootTimer = new Timer();
    private boolean shooting = false;



    // Auto shoot sequence state
    //private boolean shooting = false;
    private boolean yEdgeSent = false;
    private long shootStartMs = 0;

    private static final long MIN_SPINUP_MS = 600; // tune
    private static final double READY_ERR_RPM = 200; // matches your shooter light logic

    private int autoPatternTag = 0; // 0 = no pattern
    private int fieldPos = 0;

    private enum TurretPhase { PREAIM_FIELD90, HOMING_TO_ZERO, TRACKING }
    private TurretPhase turretPhase = TurretPhase.PREAIM_FIELD90;

    private static final double TURRET_ZERO_TOL_DEG = 3.0;
    public static double START_FIELD_HEADING_DEG = 90.0;

    // Pattern-tag board location (Pedro coords)
    private static final double PATTERN_TAG_X = 72.0;
    private static final double PATTERN_TAG_Y = 160.0;



    // ============================
    // ===== PATH BUILDING ========
    // ============================
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goPrePickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1Pose.getHeading())
                .build();

        creepToPickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup1Pose, pickup1Pose)))
                .setLinearHeadingInterpolation(prePickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup1Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        goPrePickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2Pose.getHeading())
                .build();

        creepToPickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup2Pose, pickup2Pose)))
                .setLinearHeadingInterpolation(prePickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup2Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        goPrePickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3Pose.getHeading())
                .build();

        creepToPickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup3Pose, pickup3Pose)))
                .setLinearHeadingInterpolation(prePickup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup3Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    // ============================
    // ===== STATE MACHINE ========
    // ============================
    public void autonomousPathUpdate() {
        switch (pathState) {

            // =====================
            // PRELOAD SCORE + SHOOT
            // =====================
            case 0:
                follower.followPath(scorePreload);
                setPathState(90);
                break;

            case 90:
                if (!follower.isBusy()) {
                    // We are at score pose. Ensure turret phase is homing, then wait for 0.
                    turretPhase = TurretPhase.HOMING_TO_ZERO;
                    setPathState(1);
                }

            case 1:
                if (Math.abs(turret.getCurrentAngleDeg()) < 3.0) {
                    startShootSequence();
                    setPathState(100);
                }
                break;

            case 100:
                if (updateShootSequence()) {
                    follower.followPath(goPrePickup1, PWR_FAST, false);
                    setPathState(11);
                }
                break;

            // =====================
            // PICKUP 1 -> SCORE -> SHOOT
            // =====================
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup1, PWR_CREEP, true); // hold at pickup
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true); // go score
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(101);
                }
                break;

            case 101:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goPrePickup2, PWR_FAST, false);
                    setPathState(21);
                }
                break;

            // =====================
            // PICKUP 2 -> SCORE -> SHOOT
            // =====================
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup2, PWR_CREEP, true); // hold at pickup
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true); // go score
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(102);
                }
                break;

            case 102:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goPrePickup3, PWR_FAST, false);
                    setPathState(31);
                }
                break;

            // =====================
            // PICKUP 3 -> SCORE -> SHOOT
            // =====================
            case 31:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup3, PWR_CREEP, true); // hold at pickup
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true); // go score
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(103);
                }
                break;

            case 103:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    setPathState(-1); // done
                }
                break;

            case -1:
            default:
                // done / do nothing
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ============================
    // ===== TURRET TRACKING ======
    // ============================
    private void updateTurretControl() {
        Pose robotPose = follower.getPose();

        boolean reachedShootPose = atPose(robotPose, scorePose, SCOREPOSE_POS_TOL_IN, SCOREPOSE_HEAD_TOL_DEG);
        boolean seenTag = tagSeen();

        switch (turretPhase) {
            case PREAIM_FIELD90:
                // Instead of "face field 90°", actively face the pattern-tag board point.
                // This maximizes your chance of seeing 21/22/23 while driving.
                turret.faceTarget(PATTERN_TAG_X, PATTERN_TAG_Y, robotPose);

                // As soon as we see ANY tag (pattern or goal) OR we reached the shoot pose,
                // switch to homing turret to zero.
                if (seenTag || reachedShootPose) {
                    turretPhase = TurretPhase.HOMING_TO_ZERO;
                }
                break;

            case HOMING_TO_ZERO:
                // Hard-home turret to 0 and keep it there
                turret.goToAngle(0.0);

                // Optional: once homed, allow tracking (only if a goal tag is actually visible)
                if (turretAtZero() && ENABLE_TURRET_TRACKING && !Double.isNaN(vision.getGoalTxDegOrNaN())) {
                    turretPhase = TurretPhase.TRACKING;
                }
                break;

            case TRACKING:
                // Track best goal tag (20/24) by tx
                double tx = vision.getGoalTxDegOrNaN();

                if (Double.isNaN(tx) || Math.abs(tx) < TRACK_TX_DEADBAND) {
                    // hold position
                    turret.goToAngle(turret.getCurrentAngleDeg());
                } else {
                    turret.goToAngle(turret.getCurrentAngleDeg() + TRACK_SIGN * tx);
                }
                break;
        }
    }



    private static double wrapDeg(double a) {
        while (a >= 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private static double wrapRad(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private boolean atPose(Pose cur, Pose target, double posTolIn, double headTolDeg) {
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        double dist = Math.hypot(dx, dy);

        double dhRad = wrapRad(cur.getHeading() - target.getHeading());
        double dhDeg = Math.toDegrees(dhRad);

        return dist <= posTolIn && Math.abs(dhDeg) <= headTolDeg;
    }

    /** Turret angle needed so camera/turret faces a fixed FIELD direction. */
    private double turretAngleToFaceFieldDeg(double fieldDeg) {
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        return wrapDeg(fieldDeg - robotHeadingDeg);
    }

    private void startShootSequence() {
        shooting = true;
        shootState = 0;
        shootTimer.resetTimer();
    }

    private boolean updateShootSequence() {
        if (!shooting) return true; // not shooting => "done"

        switch (shootState) {
            case 0:
                // shooter.setRpm(SOME_RPM);
                // turret.goToAngle(... optional hold/aim ...)
                shootState = 1;
                shootTimer.resetTimer();
                break;

            case 1:
                // wait for spinup (or check shooter.atSpeed())
                if (shootTimer.getElapsedTimeSeconds() > 0.5 /* or shooter.atSpeed() */) {
                    // loader.fire();  or spindexer.feedOne(); etc
                    shootState = 2;
                    shootTimer.resetTimer();
                }
                break;

            case 2:
                // wait for shot cycle to finish (servo return, index settle)
                if (shootTimer.getElapsedTimeSeconds() > 0.35) {
                    // loader.reset(); etc
                    shooting = false;
                    return true; // done shooting
                }
                break;
        }
        return false; // still shooting
    }

//    private void startShootSequence() {
//        shooting = true;
//        yEdgeSent = false;
//        shootStartMs = System.currentTimeMillis();
//    }

    private boolean shooterAtSpeed() {
        double target = shooter.getTargetRpm();
        double cur = shooter.getCurrentRpmEstimate();
        return Math.abs(target - cur) < READY_ERR_RPM;
    }

    /**
     * @param fieldPos 0 near, 1 far
     * @param tagOverride 0/21/22/23 like TeleOp driverPatternTag
     * @return true when shooting is complete
     */
    private boolean updateShootSequence(int fieldPos, int tagOverride) {
        if (!shooting) return true;

        long now = System.currentTimeMillis();

        // keep loader updated
        loader.updateLoader();

        // keep spindexer running
        boolean allowFire = shooterAtSpeed() || (now - shootStartMs) > MIN_SPINUP_MS;

        boolean yEdge = false;
        if (allowFire && !yEdgeSent) {
            yEdge = true;     // EXACTLY one loop
            yEdgeSent = true;
        }


        // IMPORTANT: keep shooter ON during the entire eject sequence
        shooter.update(true, false, false, fieldPos);

        spindexer.update(telemetry, loader, yEdge, autoPatternTag);

        // Done when we have already triggered yEdge and spindexer finished ejecting
        if (yEdgeSent && !spindexer.isEjecting()) {
            shooter.update(false, false, false, fieldPos); // stop shooter cleanly
            shooting = false;
            return true;
        }

        return false;
    }

    private boolean tagSeen() {
        // Pattern tags 21/22/23 (for spindexer)
        if (vision.getPatternTag() != 0) return true;

        // Goal tags 20/24 (for aiming)
        return !Double.isNaN(vision.getGoalTxDegOrNaN());
    }

    private boolean turretAtZero() {
        return Math.abs(turret.getCurrentAngleDeg()) <= TURRET_ZERO_TOL_DEG;
    }

    private void updateAutoPatternTag() {
        if (autoPatternTag != 0) return;         // already latched
        int t = vision.getPatternTag();          // 21/22/23 or 0
        if (t == 21 || t == 22 || t == 23) {
            autoPatternTag = t;                  // latch once
        }
    }


    // ============================
    // ===== OPMODE LIFECYCLE =====
    // ============================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter   = new ShooterSubsystemPIDF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem_State_new(hardwareMap);


        follower.setStartingPose(startPose);
        follower.update();

        buildPaths();

        telemetry.addLine("ExampleAuto ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // optional: show tags while waiting
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        spindexer.homeToIntake();
        turretPhase = TurretPhase.PREAIM_FIELD90;
    }

    @Override
    public void loop() {
        follower.update();

        //get shooting pattern
        updateAutoPatternTag();
        // keep turret tracking the entire time (optional toggle at top)
        updateTurretControl();
        turret.update();

        drawRobotOnPanels(follower.getPose());
        autonomousPathUpdate();
        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();

        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("tx(tag " + TRACK_TAG_ID + ")", "%.2f", vision.getTagTxDegOrNaN(TRACK_TAG_ID));
        telemetry.addData("AutoPatternTag", autoPatternTag);
        telemetry.update();
    }

    @Override
    public void stop() {
        // safety stop
        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();
        if (turret != null) turret.goToAngle(turret.getCurrentAngleDeg());
        ;
    }

    // ============================
    // ===== PANELS DRAWING =======
    // ============================
    private void drawRobotOnPanels(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) return;

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(ROBOT_RADIUS);

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.line(pose.getX() + v.getXComponent(), pose.getY() + v.getYComponent());

        panelsField.update();
    }
}

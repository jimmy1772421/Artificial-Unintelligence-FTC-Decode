package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new_Incremental;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemIncremental;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;


@Configurable
@Autonomous(name = "RedAutoUp_Incremental", group = "Auto")
public class RedAutoUp extends OpMode {

    // =========================
    // ===== POSES / PATHS =====
    // =========================
    private final Pose startPose   = new Pose(108, 133.32, Math.toRadians(90));
    private final Pose scorePose   = new Pose(88, 88, Math.toRadians(45));

    private final Pose prePickup1Pose = new Pose(92, 83.5, Math.toRadians(0));
    private final Pose pickup1Pose    = new Pose(130, 83.5, Math.toRadians(0));

    private final Pose prePickup2Pose = new Pose(92, 59, Math.toRadians(0));
    private final Pose pickup2Pose    = new Pose(130, 59, Math.toRadians(0));

    private final Pose prePickup3Pose = new Pose(92, 35, Math.toRadians(0));
    private final Pose pickup3Pose    = new Pose(130, 35, Math.toRadians(0));

    private Path scorePreload;
    private PathChain goPrePickup1, creepToPickup1, scorePickup1;
    private PathChain goPrePickup2, creepToPickup2, scorePickup2;
    private PathChain goPrePickup3, creepToPickup3, scorePickup3;

    // Power in path
    public static double PWR_FAST  = 0.85;
    public static double PWR_CREEP = 0.35;

    // =========================
    // ===== TURRET / AIM ======
    // =========================

    // PREAIM is ONLY for reading pattern tag (21/22/23)
    public static double PATTERN_TAG_X = 72.0;
    public static double PATTERN_TAG_Y = 160.0;

    // If vision tracking is disabled, face this point (should be same as goal)
    public static double ODO_FACE_X = 136.0;
    public static double ODO_FACE_Y = 128.0;

    // Vision-based turret tracking toggle
    public static boolean VISION_TURRET_TRACKING_ENABLED = false; //if false, odo tracking

    // Which tag ID to track for aiming (goal tag)
    public static int TRACK_TAG_ID = 24;

    // TeleOp-like tracking behavior
    public static double TRACK_TX_DEADBAND_DEG = 0.4;   // tx deadband
    public static double TRACK_MAX_STEP_DEG    = 2.0;   // max step per loop
    public static double TRACK_SIGN            = 1.0;   // flip if wrong direction

    // If tag disappears during tracking, "spin back" to zero after timeout
    public static boolean RETURN_TO_ZERO_ON_TAG_LOST = true;
    public static long TAG_LOST_TIMEOUT_MS = 250;

    // Aim-ready thresholds (used to decide when to fire)
    public static double AIM_TX_READY_DEG   = 0.7;  // when |tx| < this, consider aimed
    public static double AIM_ANGLE_TOL_DEG  = 3.0;  // when |target-current| < this in ODO mode

    // turret zero tolerance
    public static double TURRET_ZERO_TOL_DEG = 3.0;

    // how close to scorePose counts as "arrived"
    public static double SCOREPOSE_POS_TOL_IN = 2.0;
    public static double SCOREPOSE_HEAD_TOL_DEG = 4.0;

    public static double PREAIM_MAX_SEC = 1.2; // safety so preaim never deadlocks

    private enum TurretPhase {
        PREAIM_PATTERN,
        HOMING_TO_ZERO,
        VISION_TRACKING,
        ODO_FACE_POINT
    }
    private TurretPhase turretPhase = TurretPhase.PREAIM_PATTERN;

    private boolean turretZeroLatched = false;
    private long lastAimTagSeenMs = 0;

    // --- Fail-safes so first shot can’t deadlock ---
    public static long TURRET_HOME_TIMEOUT_MS = 900;  // if turret doesn't hit 0 by this, force-latch
    public static long MAX_AIM_WAIT_MS        = 1400; // if aim never "ready", force-fire after this

    private long turretHomeStartMs = 0;


    // =========================
    // ===== SHOOT SEQUENCE =====
    // =========================
    private boolean shooting = false;
    private boolean yEdgeSent = false;
    private long shootStartMs = 0;

    public static long MIN_SPINUP_MS = 600;       // tune
    public static double READY_ERR_RPM = 200;     // tune

    private boolean ejectEverStarted = false;

    // pattern tag latched from vision (21/22/23) once per match
    private int autoPatternTag = 0; // 0 = no pattern
    private int fieldPos = 0;       // 0 near, 1 far (if your shooter uses this)
    public static long SHOOT_SEQUENCE_TIMEOUT_MS = 9000; // big enough, prevents deadlock
    public static int MAX_REEJECT_ATTEMPTS = 2;

    private long shootDeadlineMs = 0;
    private int reEjectAttempts = 0;


    // =========================
    // ===== PEDRO / PANELS =====
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private FieldManager panelsField;
    private static final double ROBOT_RADIUS = 9;

    // =========================
    // ===== SUBSYSTEMS =========
    // =========================
    private TurretSubsystemIncremental turret;
    private VisionSubsystem vision;
    private ShooterSubsystemFF shooter;
    private LoaderSubsystem loader;
    private SpindexerSubsystem_State_new_Incremental spindexer;
    private IntakeSubsystem_Motor intake;

    // =========================
    // ===== PATH BUILDING ======
    // =========================
    private void buildPaths() {
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

    // =========================
    // ===== PATH STATE MACHINE ==
    // =========================
    private void autonomousPathUpdate() {
        switch (pathState) {

            // =====================
            // PRELOAD SCORE + SHOOT
            // =====================
            case 0:
                follower.followPath(scorePreload);
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    startShootSequence();   // will home turret + aim logic internally
                    setPathState(100);
                }
                break;

            case 100:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
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
                    follower.followPath(scorePickup1, true);
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
                    follower.followPath(creepToPickup2, PWR_CREEP, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
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
                    follower.followPath(creepToPickup3, PWR_CREEP, true);
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
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
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                // done
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // =========================
    // ===== PATTERN TAG LATCH ==
    // =========================
    private void updateAutoPatternTag() {
        if (autoPatternTag != 0) return;
        int t = vision.getPatternTag(); // 21/22/23 or 0
        if (t == 21 || t == 22 || t == 23) autoPatternTag = t;
    }

    // =========================
    // ===== TURRET CONTROL =====
    // =========================
    private void updateTurretControl() {
        Pose robotPose = follower.getPose();
        long now = System.currentTimeMillis();

        boolean reachedShootPose = atPose(robotPose, scorePose, SCOREPOSE_POS_TOL_IN, SCOREPOSE_HEAD_TOL_DEG);

        // Pattern tags 21/22/23
        int ptag = vision.getPatternTag();
        boolean patternSeen = (ptag == 21 || ptag == 22 || ptag == 23);

        // Aim tag tx (goal aiming)
        double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);
        boolean txValid = !Double.isNaN(tx);
        if (txValid) lastAimTagSeenMs = now;

        switch (turretPhase) {

            case PREAIM_PATTERN: {
                // Only purpose: face pattern-board so we can read 21/22/23
                turret.faceTarget(PATTERN_TAG_X, PATTERN_TAG_Y, robotPose);

                // Do NOT deadlock in preaim
                boolean preaimTimedOut = pathTimer.getElapsedTimeSeconds() > PREAIM_MAX_SEC;

                // Once we are near score pose OR we have the pattern OR timeout, go home turret
                if (reachedShootPose || patternSeen || autoPatternTag != 0 || preaimTimedOut) {
                    turretPhase = TurretPhase.HOMING_TO_ZERO;
                    turretZeroLatched = false;
                }
                break;
            }

            case HOMING_TO_ZERO: {
                turret.goToAngle(0.0);

                if (!turretZeroLatched && turretAtZero()) {
                    turretZeroLatched = true;
                    turretPhase = VISION_TURRET_TRACKING_ENABLED
                            ? TurretPhase.VISION_TRACKING
                            : TurretPhase.ODO_FACE_POINT;
                }
                break;
            }


            case VISION_TRACKING: {
                if (!VISION_TURRET_TRACKING_ENABLED) {
                    turretPhase = TurretPhase.ODO_FACE_POINT;
                    break;
                }

                // Tag lost => spin back to zero after timeout
                if (!txValid) {
                    if (RETURN_TO_ZERO_ON_TAG_LOST && (now - lastAimTagSeenMs) > TAG_LOST_TIMEOUT_MS) {
                        turretPhase = TurretPhase.HOMING_TO_ZERO;
                        turretZeroLatched = false;
                        turret.goToAngle(0.0);
                    } else {
                        turret.goToAngle(turret.getCurrentAngleDeg()); // brief hold
                    }
                    break;
                }

                // TeleOp-like: deadband + step clamp
                if (Math.abs(tx) < TRACK_TX_DEADBAND_DEG) {
                    turret.goToAngle(turret.getCurrentAngleDeg());
                } else {
                    double step = Range.clip(TRACK_SIGN * tx, -TRACK_MAX_STEP_DEG, TRACK_MAX_STEP_DEG);
                    turret.goToAngle(turret.getCurrentAngleDeg() + step);
                }
                break;
            }

            case ODO_FACE_POINT: {
                // Vision disabled => face the goal using odometry
                turret.faceTarget(ODO_FACE_X, ODO_FACE_Y, robotPose);
                break;
            }
        }
    }

    private boolean turretAtZero() {
        return Math.abs(turret.getCurrentAngleDeg()) <= TURRET_ZERO_TOL_DEG;
    }

    private boolean turretAtTarget(double tolDeg) {
        return Math.abs(turret.getTargetAngleDeg() - turret.getCurrentAngleDeg()) <= tolDeg;
    }

    // =========================
    // ===== SHOOT SEQUENCE =====
    // =========================
    private void startShootSequence() {
        shooting = true;
        yEdgeSent = false;
        shootStartMs = System.currentTimeMillis();
        ejectEverStarted = false;

        // Every time we shoot: force turret to home to 0 first, then aim (vision or ODO)
        turretPhase = TurretPhase.HOMING_TO_ZERO;
        turretZeroLatched = false;

        // start failsafe timers
        turretHomeStartMs = shootStartMs;
        shootDeadlineMs = shootStartMs + SHOOT_SEQUENCE_TIMEOUT_MS;
        reEjectAttempts = 0;

    }


    private boolean shooterAtSpeed() {
        double target = shooter.getTargetRpm();
        double cur = shooter.getCurrentRpmEstimate();
        return Math.abs(target - cur) < READY_ERR_RPM;
    }

    /**
     * @param fieldPos 0 near, 1 far
     * @param tagOverride 0/21/22/23 (pattern selection)
     * @return true when shooting is complete
     */
    private boolean updateShootSequence(int fieldPos, int tagOverride) {
        if (!shooting) return true;

        long now = System.currentTimeMillis();
        if (now > shootDeadlineMs) {
            shooting = false;     // fail-safe: don't deadlock auto forever
            return true;
        }

        // keep loader updated
        loader.updateLoader();

        // keep aiming updated
        updateTurretControl();
        turret.update();

        // Decide if we are "aim ready"
        boolean aimReady;
        if (VISION_TURRET_TRACKING_ENABLED) {
            // only fire once turret is homed AND tx is reasonably small
            double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);
            boolean txValid = !Double.isNaN(tx);
            aimReady = turretZeroLatched && (!txValid || Math.abs(tx) <= AIM_TX_READY_DEG);
            // (If you want to REQUIRE tag for aimReady, change to: txValid && Math.abs(tx) <= AIM_TX_READY_DEG)
        } else {
            // ODO fallback: fire once turret is close to its current commanded target
            aimReady = turretAtTarget(AIM_ANGLE_TOL_DEG);
        }

        // Spinup gate + aim gate
        boolean forceAimTimeout = (now - shootStartMs) > MAX_AIM_WAIT_MS;
        boolean allowFire = (shooterAtSpeed() || (now - shootStartMs) > MIN_SPINUP_MS)
                && (aimReady || forceAimTimeout);

        boolean yEdge = false;
        if (allowFire && !yEdgeSent) {
            yEdge = true;   // EXACTLY one loop
            yEdgeSent = true;
        }

        // Keep shooter ON during whole eject
        // (Main loop calls shooter.update(true,...), but safe to leave here too if you want)

        // Run spindexer eject logic
        spindexer.update(telemetry, loader, yEdge, tagOverride);

        if (spindexer.isEjecting()) ejectEverStarted = true;

        // done when eject has started and then ended
        // If eject ended but balls remain (per software), poke eject again
        if (yEdgeSent && ejectEverStarted && !spindexer.isEjecting()) {

            if (spindexer.hasAnyBall() && reEjectAttempts < MAX_REEJECT_ATTEMPTS) {
                reEjectAttempts++;

                // re-arm the one-shot yEdge (keep shooter spinning, keep aiming)
                yEdgeSent = false;
                ejectEverStarted = false;
                shootStartMs = now;            // restart spinup/aim window if you want
                shootDeadlineMs = now + SHOOT_SEQUENCE_TIMEOUT_MS;

                return false; // keep "shooting" state
            }

            // Only truly done when empty (or we've tried enough)
            if (!spindexer.hasAnyBall()) {
                shooting = false;
                return true;
            } else {
                // still not empty but we hit retry limit: let auto continue (prevents hard lock)
                shooting = false;
                return true;
            }
        }


        return false;
    }

    // =========================
    // ===== OPMODE LIFECYCLE ===
    // =========================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        turret    = new TurretSubsystemIncremental(hardwareMap);
        vision    = new VisionSubsystem(hardwareMap);
        shooter   = new ShooterSubsystemFF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem_State_new_Incremental(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);

        follower.setStartingPose(startPose);
        follower.update();
        buildPaths();

        // You manually rotate so slot0 is at LOAD when you press start:
        spindexer.homeSlot0AtLoadHere();
        // Preload preset so first shoot sequence actually has balls to eject
        spindexer.presetSlots(
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE
        );


        telemetry.addLine("RedAutoUp_Incremental ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // show tags while waiting
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("PatternTag", vision.getPatternTag());
        telemetry.addData("Aim tx(tag " + TRACK_TAG_ID + ")", "%.2f", vision.getTagTxDegOrNaN(TRACK_TAG_ID));

        telemetry.addData("Spd angle", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("Spd target", "%.1f", spindexer.getTargetAngleDeg());
        telemetry.addData("Spd intakeIdx", spindexer.getIntakeSlotIndex());
        telemetry.addData("Spd full", spindexer.isFull());
        telemetry.addData("Spd anyBall", spindexer.hasAnyBall());
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        // Start in pattern preaim
        turretPhase = TurretPhase.PREAIM_PATTERN;
        turretZeroLatched = false;

        // Start spindexer holding where it is (constructor does that)
    }

    @Override
    public void loop() {
        // Keep shooter spinning the whole match (simple + reliable in auto)
        shooter.update(true, false, false, fieldPos);

        follower.update();

        // latch pattern tag (21/22/23) once
        updateAutoPatternTag();

        // turret control always running
        updateTurretControl();
        turret.update();

        // keep loader alive
        loader.updateLoader();

        // keep spindexer alive even when not shooting
        if (!shooting) {
            spindexer.update(telemetry, loader, false, autoPatternTag);
        }

        // run path state machine
        autonomousPathUpdate();

        // intake always on (matches your TeleOp right now)
        intake.startIntake();

        drawRobotOnPanels(follower.getPose());

        // record pose for TeleOp continuity
        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();

        // telemetry
        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("AutoPatternTag", autoPatternTag);
        telemetry.addData("TurretPhase", turretPhase);
        telemetry.addData("TurretAngle", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("TurretTarget", "%.1f", turret.getTargetAngleDeg());

        telemetry.addData("Shooting", shooting);
        telemetry.addData("yEdgeSent", yEdgeSent);
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("Spd curAng", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("Spd tgtAng", "%.1f", spindexer.getTargetAngleDeg());

        double txDbg = vision.getTagTxDegOrNaN(TRACK_TAG_ID);

        telemetry.addData("DBG/shooting", shooting);
        telemetry.addData("DBG/yEdgeSent", yEdgeSent);
        telemetry.addData("DBG/turretPhase", turretPhase);
        telemetry.addData("DBG/turretZeroLatched", turretZeroLatched);
        telemetry.addData("DBG/turretAngle", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("DBG/txValid", !Double.isNaN(txDbg));
        telemetry.addData("DBG/tx", "%.2f", txDbg);
        telemetry.addData("DBG/shotAgeMs", (System.currentTimeMillis() - shootStartMs));

        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();

        if (turret != null) turret.goToAngle(turret.getCurrentAngleDeg());
        if (intake != null) intake.stopIntake();
    }

    // =========================
    // ===== PANELS DRAWING =====
    // =========================
    private void drawRobotOnPanels(Pose pose) {
        if (pose == null
                || Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading())) return;

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(ROBOT_RADIUS);

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.line(pose.getX() + v.getXComponent(), pose.getY() + v.getYComponent());

        panelsField.update();
    }

    // =========================
    // ===== POSE HELPERS =======
    // =========================
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
}

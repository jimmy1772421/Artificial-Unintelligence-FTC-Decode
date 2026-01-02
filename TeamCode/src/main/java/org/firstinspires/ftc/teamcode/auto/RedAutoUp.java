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
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

@Autonomous(name = "Example Auto", group = "Examples")
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

    // ============================
    // ===== POSES / PATHS ========
    // ============================
    private final Pose startPose   = new Pose(109, 134, Math.toRadians(90));
    private final Pose scorePose   = new Pose(87, 87, Math.toRadians(45));
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
                .addPath(new Path(new BezierLine(prePickup1Pose, pickup1Pose)))
                .setLinearHeadingInterpolation(prePickup2Pose.getHeading(), pickup2Pose.getHeading())
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
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(goPrePickup1, PWR_FAST, false);
                    setPathState(11); // new state
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup1, PWR_CREEP, true); // hold at pickup
                    setPathState(2); // then continue with your normal “scorePickup1”
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goPrePickup2, PWR_FAST, false);
                    setPathState(11); // new state
                }
                break;

            case 31:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup2, PWR_CREEP, true); // hold at pickup
                    setPathState(2); // then continue with your normal “scorePickup1”
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(goPrePickup3, PWR_FAST, false);
                    setPathState(11); // new state
                }
                break;

            case 51:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup3, PWR_CREEP, true); // hold at pickup
                    setPathState(2); // then continue with your normal “scorePickup1”
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;

            default:
                // do nothing
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

        // Before first shooting pose: hold turret facing 90° (optional)
        boolean reachedScorePose = atPose(follower.getPose(), scorePose, SCOREPOSE_POS_TOL_IN, SCOREPOSE_HEAD_TOL_DEG);

        if (HOLD_TURRET_90_UNTIL_SCOREPOSE && !reachedScorePose) {
            // IMPORTANT: do NOT run vision tracking yet
            if (HOLD_FIELD_CENTRIC) {
                double tgt = turretAngleToFaceFieldDeg(HOLD_FIELD_DEG);
                turret.goToAngle(tgt);
            } else {
                turret.goToAngle(HOLD_TURRET_DEG);
            }
            return;
        }

        // After scorePose: run your normal tracking (optional toggle at top)
        if (!ENABLE_TURRET_TRACKING) {
            turret.setManualPower(0.0);
            return;
        }

        double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);

        if (Double.isNaN(tx) || Math.abs(tx) < TRACK_TX_DEADBAND) {
            turret.setManualPower(0.0);
            return;
        }

        double power = TRACK_SIGN * TRACK_KP * tx;
        power = Range.clip(power, -TRACK_MAX_POWER, TRACK_MAX_POWER);
        turret.setManualPower(power);
    }


    private static double wrapDeg(double a) {
        while (a >= 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private boolean atPose(Pose cur, Pose target, double posTolIn, double headTolDeg) {
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        double dist = Math.hypot(dx, dy);

        double dhDeg = Math.toDegrees(wrapDeg(cur.getHeading() - target.getHeading()));
        return dist <= posTolIn && Math.abs(dhDeg) <= headTolDeg;
    }

    /** Turret angle needed so camera/turret faces a fixed FIELD direction. */
    private double turretAngleToFaceFieldDeg(double fieldDeg) {
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        return wrapDeg(fieldDeg - robotHeadingDeg);
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
    }

    @Override
    public void loop() {
        follower.update();

        // keep turret tracking the entire time (optional toggle at top)
        updateTurretControl();

        turret.update();

        drawRobotOnPanels(follower.getPose());
        autonomousPathUpdate();
        PoseStorage.lastPose = follower.getPose();

        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("tx(tag " + TRACK_TAG_ID + ")", "%.2f", vision.getTagTxDegOrNaN(TRACK_TAG_ID));
        telemetry.update();
    }

    @Override
    public void stop() {
        // safety stop
        PoseStorage.lastPose = follower.getPose();
        if (turret != null) turret.setManualPower(0.0);
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

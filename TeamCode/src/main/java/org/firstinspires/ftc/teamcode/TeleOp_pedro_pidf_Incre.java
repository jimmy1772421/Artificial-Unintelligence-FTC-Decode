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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new_Incremental;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemIncremental;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp with pedro and pidf _ incremental", group = "Test")
public class TeleOp_pedro_pidf_Incre extends OpMode {

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
    private SpindexerSubsystem_State_new_Incremental spindexer;
    private LoaderSubsystem loader;
    private TurretSubsystemIncremental turret;
    private VisionSubsystem vision;

    // ===== DASHBOARD =====
    private FtcDashboard dashboard;

    // ===== STATE VARIABLES =====
    private boolean prev2a = false;

    private int fieldPos = 0; // 0 near, 1 far
    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // turret preset edges
    private boolean prevA = false, prevDpadL = false, prevDpadR = false;

    private long shooterSpinDownDeadline = 0;
    private boolean lastEjecting = false;

    // driver2 pattern select
    private int driverPatternTag = 0;
    private boolean prev2Up = false, prev2Right = false, prev2Left = false, prev2Down = false;

    // edges
    private boolean prevLeftStick = false;
    private boolean prevRehome = false;
    private boolean prevY = false;
    private boolean prevB1 = false;

    // turret mode cycle edge
    private boolean prevModeX = false;

    // ===== SHOOT POSES (tune these numbers) =====
    public static Pose SHOOT_POSE_NEAR = new Pose(80, 80, Math.toRadians(45));
    public static Pose SHOOT_POSE_FAR  = new Pose(84, 16, Math.toRadians(65));

    // ===== SHOOT ASSIST STATE =====
    private boolean shootAssistActive = false;
    private boolean prevShootAssistBtn = false;
    private Pose activeShootPose = SHOOT_POSE_NEAR;

    private boolean autoFirePulse = false;
    private long autoFireCooldownUntil = 0;

    // ===== TURRET OWNER STATE =====
    private boolean turretPositionCommandActive = false;

    // ===== Vision->Turret tracking =====
    public static double TX_DEADBAND_DEG = 0.5;
    public static double TX_MAX_STEP_DEG = 2.0;
    public static double TX_SIGN = 1.0;
    public static double TURRET_ANGLE_TOL_DEG = 3.0;

    private enum TurretAimMode { MANUAL_HOLD, VISION_TRACK, ODO_FACE_POINT }
    private TurretAimMode turretAimMode = TurretAimMode.MANUAL_HOLD;

    // Face point in ODO mode (Pedro coords)
    public static double ODO_FACE_X = 132.0;
    public static double ODO_FACE_Y = 136.0;

    // ===== POST-PATH ACTIONS =====
    private boolean postPathActionsDone = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        Drawing.init();

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
        spindexer = new SpindexerSubsystem_State_new_Incremental(hardwareMap);
        turret    = new TurretSubsystemIncremental(hardwareMap);
        vision    = new VisionSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("TeleOp with Pedro + Shooter PIDF (Incremental)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        shooterSpinDownDeadline = 0;

        // Do NOT auto-home if you want Auto -> TeleOp continuity.
        // Just hold turret where it already is.
        turret.goToAngle(turret.getCurrentAngleDeg());
        turretPositionCommandActive = true;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // ===== UPDATE PEDRO FOLLOWER =====
        follower.update();
        telemetryM.update();

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
            follower.setTeleOpDrive(
                    -leftY * driveScale,
                    -leftX * driveScale,
                    -rightX * driveScale,
                    true
            );
        }

        // ===== SHOOT ASSIST TOGGLE (gamepad2.y) =====
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
            follower.startTeleopDrive();
            automatedDrive = false;

            if (!postPathActionsDone) {
                turret.goToAngle(0.0);
                turretPositionCommandActive = true;
                postPathActionsDone = true;
            }
        }

        // ===== Y edge (user OR auto-fire pulse) =====
        boolean yUser = gamepad1.y;
        boolean yEdgeUser = yUser && !prevY;
        prevY = yUser;

        boolean yEdge = yEdgeUser || autoFirePulse;
        autoFirePulse = false;

        // ===== FIELD POS TOGGLE (gamepad1 left stick button) =====
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !prevLeftStick) fieldPos = (fieldPos == 0) ? 1 : 0;
        prevLeftStick = leftStickButton;

        // ===== TURRET MODE CYCLE (gamepad1.x) =====
        boolean modeX = gamepad1.x;
        boolean modeEdge = modeX && !prevModeX;
        prevModeX = modeX;

        if (modeEdge) {
            switch (turretAimMode) {
                case MANUAL_HOLD:    turretAimMode = TurretAimMode.VISION_TRACK; break;
                case VISION_TRACK:   turretAimMode = TurretAimMode.ODO_FACE_POINT; break;
                case ODO_FACE_POINT: turretAimMode = TurretAimMode.MANUAL_HOLD; break;
            }
        }

        // ===== PRESET ANGLES =====
        boolean a  = gamepad1.a;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        boolean aEdge  = a  && !prevA;
        boolean dlEdge = dl && !prevDpadL;
        boolean drEdge = dr && !prevDpadR;

        prevA = a;
        prevDpadL = dl;
        prevDpadR = dr;

        if (aEdge)  { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(0.0);    turretPositionCommandActive = true; }
        if (dlEdge) { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(-90.0); turretPositionCommandActive = true; }
        if (drEdge) { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(90.0);  turretPositionCommandActive = true; }

        // ===== TURRET MANUAL/TRACKING =====
        double stickX = gamepad1.right_stick_x;
        boolean manualActive = Math.abs(stickX) > 0.05;

        if (manualActive) {
            turretAimMode = TurretAimMode.MANUAL_HOLD;
            turretPositionCommandActive = false;
            turret.setManualPower(stickX * 0.5);
        } else {
            switch (turretAimMode) {
                case VISION_TRACK: {
                    double tx = vision.getGoalTxDegOrNaN();
                    if (!Double.isNaN(tx) && Math.abs(tx) > TX_DEADBAND_DEG) {
                        double step = Range.clip(TX_SIGN * tx, -TX_MAX_STEP_DEG, TX_MAX_STEP_DEG);
                        turret.goToAngle(turret.getCurrentAngleDeg() + step);
                        turretPositionCommandActive = true;
                    } else {
                        turret.goToAngle(turret.getCurrentAngleDeg());
                        turretPositionCommandActive = true;
                    }
                    break;
                }

                case ODO_FACE_POINT: {
                    turret.faceTarget(ODO_FACE_X, ODO_FACE_Y, follower.getPose());
                    turretPositionCommandActive = true;
                    break;
                }

                case MANUAL_HOLD:
                default: {
                    if (turretPositionCommandActive) {
                        double err = turret.getTargetAngleDeg() - turret.getCurrentAngleDeg();
                        if (Math.abs(err) < TURRET_ANGLE_TOL_DEG) turretPositionCommandActive = false;
                    } else {
                        turret.setManualPower(0.0);
                    }
                    break;
                }
            }
        }

        turret.update();

        // ===== SPINDEXER COMMAND (GO TO INTAKE, NOT rezero) =====
        boolean rehomeButton = gamepad1.right_stick_button;
        if (rehomeButton && !prevRehome) spindexer.homeToIntake();
        prevRehome = rehomeButton;

        // ===== DRIVER2 PATTERN SELECT =====
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

        // ===== SPINDEXER UPDATE =====
        spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

        // Update loader after spindexer (spindexer may trigger loader.startCycle())
        loader.updateLoader();

        // Compute hasAnyBall locally from slots (since subsystem doesn't expose it)
        SpindexerSubsystem_State_new_Incremental.Ball[] slots = spindexer.getSlots();
        boolean hasAnyBall = false;
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] != SpindexerSubsystem_State_new_Incremental.Ball.EMPTY) {
                hasAnyBall = true;
                break;
            }
        }

        // ===== Eject / shooter timing =====
        boolean ejecting = spindexer.isEjecting();
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

        // ===== SHOOTER UPDATE =====
        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );

        // ===== INTAKE UPDATE =====
        if (intakeOn) intake.startIntake();
        else intake.stopIntake();

        // ===== AUTOFIRE PULSE (shoot assist) =====
        if (shootAssistActive) {
            boolean turretAtZero = Math.abs(turret.getCurrentAngleDeg()) < TURRET_ANGLE_TOL_DEG;
            boolean arrived = atPose(follower.getPose(), activeShootPose, 2.0, 6.0);

            double targetRpm = shooter.getTargetRpm();
            double currentRpm = shooter.getCurrentRpmEstimate();
            boolean rpmReady = Math.abs(targetRpm - currentRpm) < 150;

            if (postPathActionsDone && turretAtZero && arrived && rpmReady && now > autoFireCooldownUntil) {
                autoFirePulse = true;
                autoFireCooldownUntil = now + 800;
            }
        }

        // record pose
        PoseStorage.lastPose = follower.getPose();

        // ===== TELEMETRY =====
        boolean readyForIntake = !ejecting; // since subsystem doesn't expose isAutoRotating()

        int estimatedIntakeSlot = estimateIntakeSlotFromAngle(spindexer.getCurrentAngleDeg());

        telemetry.addData("Spd estIntakeSlot", estimatedIntakeSlot);
        telemetry.addData("Spd readyForIntake", readyForIntake);
        telemetry.addData("Spd full", spindexer.isFull());
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("Spd slot[0]", slots[0]);
        telemetry.addData("Spd slot[1]", slots[1]);
        telemetry.addData("Spd slot[2]", slots[2]);
        telemetry.addData("Spd angle(enc)", "%.1f", spindexer.getCurrentAngleDeg());

        double target = shooter.getTargetRpm();
        double current = shooter.getCurrentRpmEstimate();
        double error = target - current;

        telemetry.addData("Shooter On", shooter.isOn());
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Field Pos", (fieldPos == 0) ? "NEAR" : "FAR");
        telemetry.addData("Target RPM", "%.0f", target);
        telemetry.addData("Current RPM (est)", "%.0f", current);
        telemetry.addData("RPM Error", "%.0f", error);

        telemetry.addData("Pattern Tag", driverPatternTag);
        telemetry.addData("Pattern", patternStringForTag(driverPatternTag));

        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("Goal tx", "%.2f", vision.getGoalTxDegOrNaN());

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("TurretMode", turretAimMode);
        telemetry.addData("TurretAngle", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("TurretTarget", "%.1f", turret.getTargetAngleDeg());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rpm", current);
        packet.put("target", target);
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    private PathChain buildLineToPose(Pose target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, target)))
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
        postPathActionsDone = false;
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

    // ===== Helpers to replace missing spindexer methods =====

    private static int estimateIntakeSlotFromAngle(double angleDeg0to360) {
        // nearest of 0,120,240
        double a = angleDeg0to360 % 360.0;
        if (a < 0) a += 360.0;

        int idx = (int) Math.round(a / 120.0) % 3;
        if (idx < 0) idx += 3;
        return idx;
    }

    private static String patternStringForTag(int tag) {
        switch (tag) {
            case 23: return "P-P-G";
            case 22: return "P-G-P";
            case 21: return "G-P-P";
            case 0:  return "Fast (no pattern)";
            default: return "Unknown(tag=" + tag + ")";
        }
    }
}

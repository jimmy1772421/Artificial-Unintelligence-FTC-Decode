package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Pedro Auto + Spindexer Shooter", group = "Auto")
public class PedroAutoSpindexerShooter extends LinearOpMode {

    // ===== Poses =====
    // Replace with your actual start/score/intake poses
    private final Pose startPose = new Pose(72, 120, Math.toRadians(90));
    private final Pose intakePose = new Pose(100, 60, Math.toRadians(0));
    private final Pose scorePose  = new Pose(72,  20, Math.toRadians(115));

    // ===== Pedro =====
    private Follower follower;
    private PathChain goIntake;
    private PathChain goScore;

    // ===== Subsystems =====
    private ShooterSubsystemPIDF shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private TurretSubsystem turret;
    private VisionSubsystem vision;

    // ===== Auto state =====
    private int state = 0;
    private long stateDeadlineMs = 0;

    // Spindexer / shooter behavior
    private int patternTag = 0; // 0/21/22/23
    private boolean lastShootCmd = false;
    private long shooterSpinDownDeadline = 0;
    private boolean lastEjecting = false;

    // fieldPos (0 near / 1 far) — set based on where you score from
    private int fieldPos = 0;

    @Override
    public void runOpMode() {
        // Init subsystems
        shooter   = new ShooterSubsystemPIDF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);
        vision    = new VisionSubsystem(hardwareMap);

        // Init Pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        // --- Init loop: read tag for pattern order ---
        while (!isStarted() && !isStopRequested()) {
            int p = vision.getPatternTag(); // 0/21/22/23
            if (p == 21 || p == 22 || p == 23) patternTag = p;

            telemetry.addLine("Init: scanning for motif tag 21/22/23 (pattern order)");
            telemetry.addData("PatternTag", patternTag);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Home spindexer at start
        spindexer.homeToIntake();

        // Start first action
        state = 0;
        follower.followPath(goIntake);

        while (opModeIsActive()) {
            // ===== Update Pedro =====
            follower.update();

            // ===== Auto state machine (driving + when to shoot) =====
            boolean shootCmd = false; // becomes true only when we want to trigger eject
            boolean wantShooter = false;
            boolean wantIntake = false;

            switch (state) {
                case 0: // drive to intake area, run intake
                    wantIntake = true;
                    if (!follower.isBusy()) {
                        // once arrived, optionally wait a moment to suck balls in
                        state = 1;
                        stateDeadlineMs = System.currentTimeMillis() + 800; // adjust
                    }
                    break;

                case 1: // dwell at intake a bit, still intaking
                    wantIntake = true;
                    if (System.currentTimeMillis() >= stateDeadlineMs) {
                        // go score
                        follower.followPath(goScore);
                        state = 2;
                    }
                    break;

                case 2: // drive to score, stop intake and spool shooter near end
                    // safest is: intake off while moving to score so you don’t jam
                    wantIntake = false;
                    wantShooter = true;

                    if (!follower.isBusy()) {
                        // parked at score: give shooter time to stabilize, then shoot
                        state = 3;
                        stateDeadlineMs = System.currentTimeMillis() + 350; // spin-up settle
                    }
                    break;

                case 3: // wait briefly, then trigger shoot (one edge)
                    wantShooter = true;
                    if (System.currentTimeMillis() >= stateDeadlineMs) {
                        shootCmd = true; // will become a yEdge pulse below
                        state = 4;
                    }
                    break;

                case 4: // keep shooter on until spindexer finishes ejecting
                    wantShooter = true;
                    // state transition handled by spindexer flags below
                    break;

                default:
                    // done
                    wantShooter = false;
                    wantIntake = false;
                    break;
            }

            // ===== Convert shootCmd into a one-loop yEdge pulse =====
            boolean yEdge = shootCmd && !lastShootCmd;
            lastShootCmd = shootCmd;

            // ===== Run Loader continuously =====
            loader.updateLoader();

            // ===== Run Spindexer continuously =====
            boolean spindexerIsFull = spindexer.update(telemetry, loader, yEdge, patternTag);

            boolean ejecting   = spindexer.isEjecting();
            boolean hasAnyBall = spindexer.hasAnyBall();
            long now = System.currentTimeMillis();

            // spin-down logic (same idea as your teleop)
            if (lastEjecting && !ejecting && !hasAnyBall) {
                shooterSpinDownDeadline = now + 2000;
            }
            lastEjecting = ejecting;

            // If we’re in “shooting” phase, override shooter on
            boolean shooterOn = wantShooter || ejecting || (now < shooterSpinDownDeadline) || spindexerIsFull;

            // Intake logic (don’t intake during eject)
            boolean intakeOn = wantIntake && !ejecting && !spindexerIsFull;

            // ===== Apply Intake =====
            if (intakeOn) {
                intake.startIntake();
            } else {
                // If you don’t have stopIntake(), delete this and leave it off.
                // intake.stopIntake();
                intake.startIntake(); // <-- replace with stop if available; your teleop had a bug here
            }

            // ===== Shooter update (no tuning buttons in auto) =====
            shooter.update(shooterOn, false, false, fieldPos);

            // ===== Turret (optional) =====
            // Example: hold turret forward during auto
            turret.goToAngle(0.0);
            turret.setManualPower(0.0);
            turret.update();

            // ===== Auto completion condition for shoot state =====
            if (state == 4) {
                // When spindexer is done ejecting and no balls remain, you’re done.
                if (!spindexer.isEjecting() && !spindexer.hasAnyBall()) {
                    state = 99; // done
                }
            }

            // ===== Telemetry =====
            telemetry.addData("State", state);
            telemetry.addData("PatternTag", patternTag);
            telemetry.addData("FollowerBusy", follower.isBusy());
            telemetry.addData("Spd full", spindexer.isFull());
            telemetry.addData("Spd ejecting", spindexer.isEjecting());
            telemetry.addData("ShooterOn", shooterOn);
            telemetry.addData("IntakeOn", intakeOn);
            telemetry.update();
        }
    }

    private void buildPaths() {
        goIntake = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose.getHeading())
                .build();

        goScore = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, scorePose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), scorePose.getHeading())
                .build();
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

@TeleOp(name = "Shooter Test pidF", group = "Test")
public class ShooterTestTeleOpPidf extends LinearOpMode {

    private ShooterSubsystemPIDF shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private DrivetrainSubsystem drive;
    private TurretSubsystem turret;

    private FtcDashboard dashboard;

    private boolean lastY = false;
    private boolean prev2a = false;

    private int fieldPos = 0; // 0 = nearfield, 1 = far

    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    private boolean slowMode = false;

    // turret buttons (only A + dpad L/R now)
    private boolean prevA, prevDpadL, prevDpadR;

    boolean readyForIntake = true;
    private long shooterSpinDownDeadline = 0;   // time (ms) until we turn shooter off
    private boolean lastEjecting = false;       // for edge-detect on eject end

    private int driverPatternTag = 0;  // 0 = fastest, 21/22/23 = pattern
    private boolean prev2Up, prev2Right, prev2Left, prev2Down;

    private boolean prevLeftStick = false;

    // rehome button edge tracking
    private boolean prevRehome = false;

    private boolean prev2LeftBumper = false;
    private boolean prev2RightBumper = false;

    // === PID tuning on driver gamepad ===
    // 0 = kP, 1 = kI, 2 = kD, 3 = kF
    private int pidParamIndex = 0;
    private boolean prevStart1 = false;
    private boolean prevXpid = false; // square
    private boolean prevBpid = false; // circle

    // step sizes for tuning
    private static final double KP_STEP = 0.0001;
    private static final double KI_STEP = 0.000001;
    private static final double KD_STEP = 0.00005;
    private static final double KF_STEP = 0.000001; // adjust if too coarse/fine

    @Override
    public void runOpMode() {
        shooter   = new ShooterSubsystemPIDF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        drive     = new DrivetrainSubsystem(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);

        // FTC Dashboard hookup
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Spindexer auto-intake running.");
        telemetry.addLine("Y: start eject sequence (if any balls).");
        telemetry.addLine("START: cycle PID param (kP/kI/kD/kF)");
        telemetry.addLine("SQUARE (X): - selected PIDF, CIRCLE (B): + selected PIDF");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Home spindexer so slot 0 is at intake (abs ≈ 260.7°)
        spindexer.homeToIntake();

        while (opModeIsActive()) {
            // ===== SHOOTER FIELD POSITION TOGGLE =====
            boolean leftStickButton = gamepad1.left_stick_button;

            if (leftStickButton && !prevLeftStick) {
                fieldPos = (fieldPos == 0) ? 1 : 0; // toggle near/far
            }
            prevLeftStick = leftStickButton;

            // Shooter update uses shooterOn; shooterOn decided later based on spindexer state.
            shooter.update(
                    shooterOn,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    fieldPos
            );

            // ===== DRIVETRAIN =====
            double leftX  = gamepad2.left_stick_x;
            double leftY  = gamepad2.left_stick_y;
            double rightX = gamepad2.right_stick_x;

            boolean a2 = gamepad2.a;
            if (a2 && !prev2a) {
                slowMode = !slowMode;
            }
            prev2a = a2;

            drive.setDriveScale(slowMode ? 0.4 : 1.0);
            drive.drive(leftX, leftY, rightX);

            // ===== LOADER =====
            loader.updateLoader();

            // ===== INTAKE =====
            if (intakeOn) {
                intake.startIntake();
            } else {
                intake.startIntake();
            }

            // ===== TURRET =====
            boolean a = gamepad1.a;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;

            if (a && !prevA) turret.goToAngle(0.0);
            if (dl && !prevDpadL) turret.goToAngle(90.0);
            if (dr && !prevDpadR) turret.goToAngle(-90.0);

            prevA = a;
            prevDpadL = dl;
            prevDpadR = dr;

            double stickX = gamepad1.right_stick_x;
            if (Math.abs(stickX) > 0.05) {
                turret.setManualPower(stickX * 0.4);
            } else {
                turret.setManualPower(0.0);
            }

            turret.update();

            // ===== SPINDEXER / PATTERN INPUT =====

            // Y on gamepad1 starts eject sequence
            boolean yEdge = gamepad1.y && !lastY;
            lastY = gamepad1.y;

            // Rehome spindexer on right stick button
            boolean rehomeButton = gamepad1.right_stick_button;
            if (rehomeButton && !prevRehome) {
                spindexer.homeToIntake();
            }
            prevRehome = rehomeButton;

            // driver 2 chooses pattern tag with dpad:
            boolean dUp2    = gamepad2.dpad_up;
            boolean dRight2 = gamepad2.dpad_right;
            boolean dLeft2  = gamepad2.dpad_left;
            boolean dDown2  = gamepad2.dpad_down;

            if (dUp2 && !prev2Up) {
                driverPatternTag = 23;
            }
            if (dRight2 && !prev2Right) {
                driverPatternTag = 22;
            }
            if (dLeft2 && !prev2Left) {
                driverPatternTag = 21;
            }
            if (dDown2 && !prev2Down) {
                driverPatternTag = 0; // fastest / no pattern
            }

            prev2Up    = dUp2;
            prev2Right = dRight2;
            prev2Left  = dLeft2;
            prev2Down  = dDown2;

            spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

            // --- Eject / shooter timing logic ---
            boolean ejecting   = spindexer.isEjecting();
            boolean hasAnyBall = spindexer.hasAnyBall();
            long now = System.currentTimeMillis();

            if (lastEjecting && !ejecting && !hasAnyBall) {
                shooterSpinDownDeadline = now + 2000;  // 2 s after last ball
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
            intakeOn  = wantIntake;

            // ===== MANUAL FORCE-REGISTER FOR SPINDEXER (driver 2 bumpers) =====
            boolean lb2 = gamepad2.left_bumper;
            boolean rb2 = gamepad2.right_bumper;

            if (lb2 && !prev2LeftBumper) {
                spindexer.forceIntakeSlotGreen(telemetry);
            }
            if (rb2 && !prev2RightBumper) {
                spindexer.forceIntakeSlotPurple(telemetry);
            }

            prev2LeftBumper = lb2;
            prev2RightBumper = rb2;

            // ===== PID TUNING (driver: START + SQUARE/CIRCLE) =====
            boolean start1 = gamepad1.start;
            boolean xPid   = gamepad1.x; // square
            boolean bPid   = gamepad1.b; // circle

            // cycle which param we’re editing
            if (start1 && !prevStart1) {
                pidParamIndex = (pidParamIndex + 1) % 4; // 0 -> 1 -> 2 -> 3 -> 0
            }


            // Circle (B) increases, Square (X) decreases selected param (edge-triggered)
            if (bPid && !prevBpid) {
                switch (pidParamIndex) {
                    case 0: shooter.adjustKp(KP_STEP);  break;
                    case 1: shooter.adjustKi(KI_STEP);  break;
                    case 2: shooter.adjustKd(KD_STEP);  break;
                    case 3: shooter.adjustKf(KF_STEP);  break;
                }
            }

            if (xPid && !prevXpid) {
                switch (pidParamIndex) {
                    case 0: shooter.adjustKp(-KP_STEP);  break;
                    case 1: shooter.adjustKi(-KI_STEP);  break;
                    case 2: shooter.adjustKd(-KD_STEP);  break;
                    case 3: shooter.adjustKf(-KF_STEP);  break;
                }
            }

            prevBpid = bPid;
            prevXpid = xPid;

            // ===== TELEMETRY =====
            SpindexerSubsystem.Ball[] s = spindexer.getSlots();

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

            // shooter telemetry
            double target = shooter.getTargetRpm();
            double current = shooter.getCurrentRpmEstimate();
            double error = target - current;

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Field Pos", (fieldPos == 0) ? "NEAR" : "FAR");
            telemetry.addData("Target RPM", "%.0f", target);
            telemetry.addData("Current RPM (est)", "%.0f", current);
            telemetry.addData("RPM Error", "%.0f", error);

            String selectedParamName =
                    (pidParamIndex == 0) ? "kP" :
                            (pidParamIndex == 1) ? "kI" :
                                    (pidParamIndex == 2) ? "kD" : "kF";

            telemetry.addData("PIDF Selected", selectedParamName);
            telemetry.addData("kP", shooter.getKp());
            telemetry.addData("kI", shooter.getKi());
            telemetry.addData("kD", shooter.getKd());
            telemetry.addData("kF", shooter.getKf());


            telemetry.addData("Pattern Tag", driverPatternTag);
            telemetry.addData("Pattern Order", spindexer.getGamePattern());

            // === DASHBOARD PACKET (for graphs) ===
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("rpm", current);
            packet.put("target", target);
            packet.put("error", error);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}

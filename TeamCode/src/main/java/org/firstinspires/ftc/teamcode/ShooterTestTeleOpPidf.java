package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

@TeleOp(name = "Shooter Test FF+PI Tuner", group = "Test")
public class ShooterTestTeleOpPidf extends LinearOpMode {

    private ShooterSubsystemFF shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private DrivetrainSubsystem drive;
    private TurretSubsystem turret;

    private FtcDashboard dashboard;

    private boolean lastY = false;
    private boolean prev2a = false;

    private int fieldPos = 0; // 0 = nearfield, 1 = far
    private boolean slowMode = false;

    // turret buttons
    private boolean prevA, prevDpadL, prevDpadR;

    // spindexer state
    private boolean spindexerIsFull = false;
    private boolean prev2Up, prev2Right, prev2Left, prev2Down;
    private boolean prevLeftStick = false;
    private boolean prevRehome = false;
    private boolean prev2LeftBumper = false;
    private boolean prev2RightBumper = false;

    // === TUNING ===
    // 0=P, 1=I, 2=kV, 3=kS
    private int paramIndex = 0;
    private boolean prevStart1 = false;
    private boolean prevX = false;
    private boolean prevB = false;

    private static final double P_BASE  = 1e-5;
    private static final double I_BASE  = 1e-7;
    private static final double KV_BASE = 1e-6;
    private static final double KS_BASE = 1e-3;

    private static final double[] STEP_MULT = {0.1, 1.0, 10.0, 100.0, 1000.0};
    private int stepModeIndex = 1;
    private boolean prevStepModeBtn = false;

    // Force shooter on for tuning
    private boolean forceShooterOn = true;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        shooter   = new ShooterSubsystemFF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        drive     = new DrivetrainSubsystem(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Tuning: START cycles (P/I/kV/kS).  B=+  X=-");
        telemetry.addLine("LB (gp1) cycles step scale.");
        telemetry.addLine("RB (gp1) toggles ForceShooterOn.");
        telemetry.addLine("LeftStickButton toggles near/far (hood + target RPM).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        spindexer.homeToIntake();

        while (opModeIsActive()) {

            // ===== Step scale toggle (gp1 LB) =====
            boolean stepBtn = gamepad1.left_bumper;
            if (stepBtn && !prevStepModeBtn) {
                stepModeIndex = (stepModeIndex + 1) % STEP_MULT.length;
            }
            prevStepModeBtn = stepBtn;

            double scale = STEP_MULT[stepModeIndex];
            double pStep  = P_BASE  * scale;
            double iStep  = I_BASE  * scale;
            double kvStep = KV_BASE * scale;
            double ksStep = KS_BASE * scale;

            // ===== Force shooter on toggle (gp1 RB) =====
            boolean rb = gamepad1.right_bumper;
            if (rb && !prevRB) forceShooterOn = !forceShooterOn;
            prevRB = rb;

            // ===== Field pos toggle =====
            boolean leftStickButton = gamepad1.left_stick_button;
            if (leftStickButton && !prevLeftStick) fieldPos = (fieldPos == 0) ? 1 : 0;
            prevLeftStick = leftStickButton;

            // ===== DRIVETRAIN =====
            double leftX  = gamepad2.left_stick_x;
            double leftY  = gamepad2.left_stick_y;
            double rightX = gamepad2.right_stick_x;

            boolean a2 = gamepad2.a;
            if (a2 && !prev2a) slowMode = !slowMode;
            prev2a = a2;

            drive.setDriveScale(slowMode ? 0.4 : 1.0);
            drive.drive(leftX, leftY, rightX);

            // ===== LOADER =====
            loader.updateLoader();

            // ===== INTAKE (always on) =====
            intake.startIntake();

            // ===== TURRET (same as your code) =====
            boolean a = gamepad1.a;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;

            if (a && !prevA) turret.goToAngle(0.0);
            if (dl && !prevDpadL) turret.goToAngle(90.0);
            if (dr && !prevDpadR) turret.goToAngle(-90.0);

            prevA = a; prevDpadL = dl; prevDpadR = dr;

            double stickX = gamepad1.right_stick_x;
            if (Math.abs(stickX) > 0.05) turret.setManualPower(stickX * 0.4);
            else turret.setManualPower(0.0);

            turret.update();

            // ===== SPINDEXER / PATTERN =====
            boolean yEdge = gamepad1.y && !lastY;
            lastY = gamepad1.y;

            boolean rehomeButton = gamepad1.right_stick_button;
            if (rehomeButton && !prevRehome) spindexer.homeToIntake();
            prevRehome = rehomeButton;

            int driverPatternTag = 0;
            boolean dUp2    = gamepad2.dpad_up;
            boolean dRight2 = gamepad2.dpad_right;
            boolean dLeft2  = gamepad2.dpad_left;
            boolean dDown2  = gamepad2.dpad_down;

            if (dUp2 && !prev2Up) driverPatternTag = 23;
            if (dRight2 && !prev2Right) driverPatternTag = 22;
            if (dLeft2 && !prev2Left) driverPatternTag = 21;
            if (dDown2 && !prev2Down) driverPatternTag = 0;

            prev2Up = dUp2; prev2Right = dRight2; prev2Left = dLeft2; prev2Down = dDown2;

            spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

            boolean lb2 = gamepad2.left_bumper;
            boolean rb2b = gamepad2.right_bumper;
            if (lb2 && !prev2LeftBumper) spindexer.forceIntakeSlotGreen(telemetry);
            if (rb2b && !prev2RightBumper) spindexer.forceIntakeSlotPurple(telemetry);
            prev2LeftBumper = lb2;
            prev2RightBumper = rb2b;

            // ===== TUNING INPUTS (START cycles param, B/X adjusts) =====
            boolean start1 = gamepad1.start;
            boolean xBtn = gamepad1.x;
            boolean bBtn = gamepad1.b;

            if (start1 && !prevStart1) paramIndex = (paramIndex + 1) % 4;

            if (bBtn && !prevB) {
                switch (paramIndex) {
                    case 0: shooter.adjustKp(+pStep); break;
                    case 1: shooter.adjustKi(+iStep); break;
                    case 2: shooter.adjustKv(+kvStep); break;
                    case 3: shooter.adjustKs(+ksStep); break;
                }
            }
            if (xBtn && !prevX) {
                switch (paramIndex) {
                    case 0: shooter.adjustKp(-pStep); break;
                    case 1: shooter.adjustKi(-iStep); break;
                    case 2: shooter.adjustKv(-kvStep); break;
                    case 3: shooter.adjustKs(-ksStep); break;
                }
            }

            prevStart1 = start1;
            prevB = bBtn;
            prevX = xBtn;

            // ===== SHOOTER UPDATE =====
            boolean shooterOn = forceShooterOn; // keep it steady for tuning
            shooter.update(
                    shooterOn,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    fieldPos
            );

            // ===== TELEMETRY =====
            double targetRpm = shooter.getTargetRpm();
            double curRpm = shooter.getVelocityRpm();
            double errRpm = targetRpm - curRpm;

            String paramName =
                    (paramIndex == 0) ? "P" :
                            (paramIndex == 1) ? "I" :
                                    (paramIndex == 2) ? "kV" : "kS";

            telemetry.addData("ForceShooterOn", forceShooterOn);
            telemetry.addData("Field Pos", (fieldPos == 0) ? "NEAR" : "FAR");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("RPM", "%.0f", curRpm);
            telemetry.addData("RPM Err", "%.0f", errRpm);

            telemetry.addData("Target tps", "%.1f", shooter.getTargetTps());
            telemetry.addData("Vel tps", "%.1f", shooter.getVelocityTps());
            telemetry.addData("PowerCmd", "%.3f", shooter.getPowerCmd());
            telemetry.addData("FF", "%.4f", shooter.getFF());

            telemetry.addData("Selected", paramName);
            telemetry.addData("StepScale", STEP_MULT[stepModeIndex]);

            telemetry.addData("P", "%.8f", shooter.getKp());
            telemetry.addData("I", "%.10f", shooter.getKi());
            telemetry.addData("kV", "%.10f", shooter.getKv());
            telemetry.addData("kS", "%.6f", shooter.getKs());

            // Graphs
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("rpm", curRpm);
            packet.put("target_rpm", targetRpm);
            packet.put("err_rpm", errRpm);
            packet.put("power", shooter.getPowerCmd());
            packet.put("ff", shooter.getFF());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}

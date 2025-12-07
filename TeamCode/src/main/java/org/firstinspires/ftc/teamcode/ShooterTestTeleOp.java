package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private DrivetrainSubsystem drive;
    private TurretSubsystem turret;

    private boolean lastY = false;

    private boolean prev2a = false;

    private int fieldPos = 0; // 0 = nearfield, 1 = far

    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    private boolean slowMode = false;

    // turret buttons
    private boolean prevA, prevB, prevX, prevDpadL, prevDpadR;

    boolean readyForIntake = true;
    private long shooterSpinDownDeadline = 0;   // time (ms) until we turn shooter off
    private boolean lastEjecting = false;       // for edge-detect on eject end

    private int driverPatternTag = 0;  // 0 = fastest, 21/22/23 = pattern
    private boolean prev2Up, prev2Right, prev2Left, prev2Down;

    private boolean prevLeftStick = false;

    // --- NEW: rehome button edge tracking (M1-style) ---
    // Map M1 to some button; here we use gamepad1.left_stick_button
    private boolean prevRehome = false;

    private boolean prev2LeftBumper = false;
    private boolean prev2RightBumper = false;


    @Override
    public void runOpMode() {
        shooter   = new ShooterSubsystem(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        drive     = new DrivetrainSubsystem(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);

        telemetry.addLine("Spindexer auto-intake running.");
        telemetry.addLine("Y: start eject sequence (if any balls).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Home spindexer so slot 0 is at intake (abs ≈ 260.7°)
        spindexer.homeToIntake();

        while (opModeIsActive()) {
            // ===== SHOOTER =====
            // ===== SHOOTER FIELD POSITION TOGGLE =====
            boolean leftStickButton = gamepad1.left_stick_button;

            // Rising edge: button is pressed now, but wasn't last loop
            if (leftStickButton && !prevLeftStick) {
                // Toggle between 0 (near) and 1 (far)
                fieldPos = (fieldPos == 0) ? 1 : 0;
            }

            // Update previous state
            prevLeftStick = leftStickButton;

            shooter.update(
                    shooterOn,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    fieldPos
            );

            // ==== DRIVETRAIN ====
            double leftX  = gamepad2.left_stick_x;
            double leftY  = gamepad2.left_stick_y;
            double rightX = gamepad2.right_stick_x;

            boolean a2 = gamepad2.a;
            if (a2 && !prev2a) {
                slowMode = !slowMode;
            }
            prev2a = a2;

            drive.setDriveScale(slowMode ? 0.4 : 1.0);

            telemetry.addData("Drive Scale", drive.getDriveScale());
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
            // Presets using RUN_TO_POSITION
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;


            if (a && !prevA) turret.goToAngle(0.0);
            if (b && !prevB) turret.goToAngle(45.0);
            if (x && !prevX) turret.goToAngle(-45.0);
            if (dl && !prevDpadL) turret.goToAngle(90.0);
            if (dr && !prevDpadR) turret.goToAngle(-90.0);


            prevA = a;
            prevB = b;
            prevX = x;
            prevDpadL = dl;
            prevDpadR = dr;


            // Manual override with joystick
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

            // --- NEW: M1 rehome button (here: gamepad1.left_stick_button) ---
            boolean rehomeButton = gamepad1.right_stick_button; // bind your M1 to this
            if (rehomeButton && !prevRehome) {
                // Rehome spindexer: rotate slot 0 back to intake
                spindexer.homeToIntake();
                // (Optional) you could also rezero encoder or clear slots here if you want
            }
            prevRehome = rehomeButton;

            // driver 2 chooses pattern tag with dpad:
            // up    -> 23 (P,P,G)
            // right -> 22 (P,G,P)
            // left  -> 21 (G,P,P)
            // down  -> 0  (no pattern / fastest)
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

            // Call spindexer.update with the pattern override
            spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

            // --- Eject / shooter timing logic ---
            boolean ejecting   = spindexer.isEjecting();
            boolean hasAnyBall = spindexer.hasAnyBall();
            long now = System.currentTimeMillis();

            // Detect the moment the eject sequence finishes AND all balls are gone
            if (lastEjecting && !ejecting && !hasAnyBall) {
                // keep shooter spinning for 2 seconds after last ball
                shooterSpinDownDeadline = now + 2000;  // 2000 ms = 2 s
            }
            lastEjecting = ejecting;

            // Decide whether we want shooter / intake right now
            boolean wantShooter;
            boolean wantIntake;

            // Case A: magazine full → spin up shooter, stop intake
            if (spindexerIsFull) {
                wantShooter = true;
                wantIntake  = false;

                // Case B: in the middle of ejecting OR within 2s spin-down window
            } else if (ejecting || now < shooterSpinDownDeadline) {
                wantShooter = true;
                wantIntake  = false;

                // Case C: not full, not ejecting, spin-down done → refill mag
            } else {
                wantShooter = false;
                wantIntake  = true;
            }

            shooterOn = wantShooter;
            intakeOn  = wantIntake;

            // ===== MANUAL FORCE-REGISTER FOR SPINDEXER (driver 2 bumpers) =====
            boolean lb2 = gamepad2.left_bumper;
            boolean rb2 = gamepad2.right_bumper;

// Left bumper: force current intake slot as GREEN
            if (lb2 && !prev2LeftBumper) {
                spindexer.forceIntakeSlotGreen(telemetry);
            }

// Right bumper: force current intake slot as PURPLE
            if (rb2 && !prev2RightBumper) {
                spindexer.forceIntakeSlotPurple(telemetry);
            }

// update edge-detect state
            prev2LeftBumper = lb2;
            prev2RightBumper = rb2;


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

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.addData("Pattern Tag", driverPatternTag);
            telemetry.addData("Pattern Order", spindexer.getGamePattern());

            telemetry.update();
        }
    }
}

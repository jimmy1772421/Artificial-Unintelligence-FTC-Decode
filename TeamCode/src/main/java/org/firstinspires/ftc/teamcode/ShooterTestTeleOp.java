package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
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

    private int fieldPos = 0;// 0 = nearfield, 1= far

    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    private boolean slowMode = false;

    //turret buttons
    private boolean prevA, prevB, prevX, prevDpadL, prevDpadR, prevLeftBumper, prevRightBumper;

    boolean readyForIntake = true;

    @Override
    public void runOpMode() {
        shooter   = new ShooterSubsystem(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        drive = new DrivetrainSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        telemetry.addLine("Spindexer auto-intake running.");
        telemetry.addLine("Y: start eject sequence (if any balls).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Home spindexer so slot 0 is at intake (abs ≈ 260.7°)
        spindexer.homeToIntake();

        while (opModeIsActive()) {
            // ===== SHOOTER =====
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
            drive.drive(leftX,leftY,rightX);

            // ===== LOADER (manual B) =====
            /*
            boolean b = gamepad1.b;
            if (b && !prevB) {
                loader.startCycle();
            }
            prevB = b;
            loader.updateLoader();
            */

            // ===== INTAKE =====
            if (intakeOn) {
                intake.startIntake();
            } else {
                intake.stopIntake();
            }

            // ===== SPINDEXER =====
            boolean yEdge = gamepad1.y && !lastY;
            lastY = gamepad1.y;

            spindexerIsFull= spindexer.update(telemetry, loader, yEdge);

            if (spindexerIsFull) {
                shooterOn = true;
                intakeOn = false;
            }else{
                shooterOn = false;
                intakeOn = true;
            }

            //turret
            // --- Presets using RUN_TO_POSITION ---
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;
            boolean bl = gamepad1.left_bumper;
            boolean br = gamepad1.right_bumper;

            if (a && !prevA) turret.goToAngle(0.0);
            if (b && !prevB) turret.goToAngle(45.0);
            if (x && !prevX) turret.goToAngle(-45.0);
            if (dl && !prevDpadL) turret.goToAngle(90.0);
            if (dr && !prevDpadR) turret.goToAngle(-90.0);
            if (bl && !prevLeftBumper ) turret.goToAngle(135.0);
            if (br && !prevRightBumper) turret.goToAngle(-135.0);

            prevA = a;
            prevB = b;
            prevX = x;
            prevDpadL = dl;
            prevDpadR = dr;
            prevLeftBumper = bl;
            prevRightBumper = br;

            // --- Manual override with joystick ---
            double stickX = gamepad1.right_stick_x;
            if (Math.abs(stickX) > 0.05) {
                turret.setManualPower(stickX * 0.1);  // manual mode, overrides auto
            }else{
                turret.setManualPower(stickX * 0);
            }

            turret.update();



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

            telemetry.update();
        }
    }
}

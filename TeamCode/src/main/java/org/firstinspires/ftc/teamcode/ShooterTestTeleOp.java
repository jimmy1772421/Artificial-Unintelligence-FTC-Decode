package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;
    private boolean prevB = false;

    //spindexer vars
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastBack = false;

    private boolean preparedForEject = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        LoaderSubsystem loader = new LoaderSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        telemetry.addLine("Spindexer controls:");
        telemetry.addLine("X: Intake one (read color, move to next slot)");
        telemetry.addLine("Y: Eject ALL (from 180° load position)");
        telemetry.addLine("Back: Re-zero (current angle = slot 0 at intake)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // A = toggle on/off
            // dpad_up = +250 RPM
            // dpad_down = -250 RPM
            shooter.update(
                    gamepad1.a,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down
            );
            boolean b = gamepad1.b;

            // On rising edge of A (just pressed)
            if (b && !prevB) {
                loader.startCycle();
            }
            prevB = b;

            // Let the subsystem handle timing and auto-return
            loader.updateLoader();

            intake.StartIntake();

            // ----- Spindexer inputs (X, Y, Back) -----
            boolean xEdge = gamepad1.x && !lastX;   lastX = gamepad1.x;
            boolean yEdge = gamepad1.y && !lastY;   lastY = gamepad1.y;
            boolean backEdge = gamepad1.back && !lastBack; lastBack = gamepad1.back;

            // X: Intake one
            if (xEdge) {
                spindexer.intakeOne(telemetry);   // aligns, reads color, moves to next slot
                preparedForEject = false;
            }

            // Auto-prep when 3 balls onboard
            if (!preparedForEject && spindexer.hasThreeBalls()) {
                boolean ok = spindexer.prepareFirstEjectByPattern(telemetry);
                preparedForEject = ok;
            }

            // Y: Eject all from 180° load position
            if (yEdge) {
                spindexer.ejectAllByPattern(telemetry);
                preparedForEject = false;
            }

            // Back: rezero here
            if (backEdge) {
                spindexer.rezeroHere();
                preparedForEject = false;
            }

            // ----- Telemetry -----
            SpindexerSubsystem.Ball[] s = spindexer.getSlots();
            telemetry.addData("Spd enc", spindexer.getEncoder());
            telemetry.addData("Spd target", spindexer.getTarget());
            telemetry.addData("Spd intakeIndex", spindexer.getIntakeIndex());
            telemetry.addData("Spd preparedForEject", preparedForEject);
            telemetry.addData("Spd slot[0]", s[0]);
            telemetry.addData("Spd slot[1]", s[1]);
            telemetry.addData("Spd slot[2]", s[2]);

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());

            telemetry.update();
        }
    }
}

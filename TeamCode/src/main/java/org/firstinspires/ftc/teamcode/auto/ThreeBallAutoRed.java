package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

@Autonomous(name = "ThreeBallPathAutoRed", group = "Comp")
public class ThreeBallAutoRed extends LinearOpMode {

    private DrivetrainSubsystem drive;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private IntakeSubsystem_Motor intake;

    // 0 = near field, 1 = far (you can tweak if you want far)
    private int fieldPos = 0;

    // How long to intake while driving in the middle phase (ms)
    private static final long INTAKE_PHASE_MS = 2000;

    @Override
    public void runOpMode() {
        // ---- INIT SUBSYSTEMS ----
        drive     = new DrivetrainSubsystem(hardwareMap);
        shooter   = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);

        telemetry.addLine("Auto initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Just in case: make sure spindexer is homed
        spindexer.homeToIntake();
        //shooter.setNearRPM(3500);

        // ========= PHASE 1: Move to first shooting spot =========
        // "move forwards, move side ways, then rotate"
        moveToFirstShootingPosition();

        // ========= PHASE 2: Shoot 3 preloaded balls (fast / no pattern) =========
        shootMagazineFastNoPattern();

        // ========= PHASE 3: Reposition for intake path =========
        // "I then rotate , call move forward once, move side ways once"
        repositionForIntakePath();

        // ========= PHASE 4: Intake + spindexer loop while driving =========
        runIntakeAndSpindexerLoop();

        // ========= PHASE 5: Move to second shooting position =========
        // "After the loop is done, move forward, move sideways, rotate"
        moveToSecondShootingPosition();

        // ========= PHASE 6: Shoot 3 balls again (whatever we have) =========
        shootMagazineFastNoPattern();

        // ========= END: Stop everything =========
        stopAll();
    }

    // ---------------- DRIVE WRAPPERS ----------------
    // These are where YOU plug in your actual driveForwardMm / strafe / rotate functions.

    private void moveToFirstShootingPosition() {
        // TODO: Replace with your real distances/angles
        // Example if you add driveForwardMm / strafeMm:
        // drive.driveForwardMm(this, 800, 0.5);
        // drive.strafeMm(this, 300, 0.5);
        // drive.rotateInPlaceForMs(this, 0.4, 500);
        drive.strafeMm(this, -600, 0.5);
        drive.driveForwardMm(this,1000,0.5);

        drive.rotateInPlaceForMs(this, -0.4, 500);
        // Placeholder: do nothing for now
    }

    private void repositionForIntakePath() {
        // TODO: Replace with your actual rotate + forward + sideways sequence
        // Example:
        // drive.rotateInPlaceForMs(this, -0.4, 400);
        // drive.driveForwardMm(this, 600, 0.5);
        // drive.strafeMm(this, -250, 0.5);

        // Placeholder
    }

    private void moveToSecondShootingPosition() {
        // TODO: Replace with your actual forward + sideways + rotate
        // Example:
        // drive.driveForwardMm(this, 500, 0.5);
        // drive.strafeMm(this, 200, 0.5);
        // drive.rotateInPlaceForMs(this, 0.4, 400);

        // Placeholder
    }


    private void stopAll() {
        drive.stopAll();
        intake.stopIntake();
        shooter.stop();  // hard off
        // optional: keep state in sync
        shooter.update(false, false, false, fieldPos);
    }


    // ---------------- SHOOTER HELPERS ----------------

    /**
     * Ensure shooter is ON (respects the toggle logic inside ShooterSubsystem).
     * We check shooter.isOn(), and if it's off we send a single rising edge.
     */
    private void ensureShooterOn() {
        if (!shooter.isOn()) {
            // Rising edge to toggle ON
            shooter.update(true, false, false, fieldPos);
            // Then immediately release the button
            shooter.update(false, false, false, fieldPos);
        } else {
            // Keep updating with button=false so RPM stays commanded
            shooter.update(false, false, false, fieldPos);
        }
    }

    /**
     * Ensure shooter is OFF (respects toggle logic).
     */
    private void ensureShooterOff() {
        if (shooter.isOn()) {
            // Rising edge to toggle OFF
            shooter.update(true, false, false, fieldPos);
            shooter.update(false, false, false, fieldPos);
        } else {
            shooter.update(false, false, false, fieldPos);
        }
    }

    /**
     * Spin up the shooter for a given time (ms) WITHOUT firing anything.
     * Spindexer still updates (auto-intake logic), but we do not send yEdge.
     */
    private void spinUpShooterForMs(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            ensureShooterOn();  // keep shooter running
            loader.updateLoader();

            // No eject command, just keep spindexer updated (patternTagOverride = 0 for "fast")
            spindexer.update(telemetry, loader, false, 0);

            telemetry.addData("Phase", "Spinning up shooter");
            telemetry.update();
            idle();
        }
    }

    /**
     * Shoot whatever is currently in the magazine using the spindexer state machine,
     * in "fast" mode (patternTagOverride = 0).
     *
     * Assumes:
     *  - slots[] in the spindexer accurately reflect which slots have balls
     *  - auto-intake has already loaded slots OR you preloaded and seeded slots in code
     *
     * ShooterSubsystem.update(on, rpmUp, rpmDown, fieldPos)
     *  -> HERE `on` is treated as "want shooter ON this loop", NOT a toggle.
     */
    /**
     * Shoot whatever is currently in the magazine using the spindexer state machine,
     * in "fast" mode (patternTagOverride = 0).
     *
     * Assumes:
     *  - slots[] in the spindexer accurately reflect which slots have balls
     *  - auto-intake has already loaded slots OR you preloaded and seeded slots in code
     *
     * ShooterSubsystem.update(on, rpmUp, rpmDown, fieldPos)
     *  -> HERE `on` is treated as "want shooter ON this loop", NOT a toggle.
     */
    private void shootMagazineFastNoPattern() {

        // ===== 1) Spin up shooter once and let it get to speed =====
        long spinupEnd = System.currentTimeMillis() + 1500; // 1.5s, tweak as needed

        while (opModeIsActive() && System.currentTimeMillis() < spinupEnd) {
            // Keep shooter ON the whole time
            shooter.update(true,  false, false, fieldPos);

            // Keep spindexer + loader state machines alive (no eject yet)
            spindexer.update(telemetry, loader, false, 0); // yEdge=false, tag=0 (fastest)
            loader.updateLoader();

            telemetry.addData("Auto", "Spinning up shooter...");
            telemetry.addData("Shooter RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
            idle();
        }

        // ===== 2) Fire up to 3 shots: one Y-edge per shot =====
        final int MAX_SHOTS = 3;
        for (int shot = 0; shot < MAX_SHOTS; shot++) {

            if (!opModeIsActive()) break;
            // If spindexer *thinks* we're out of balls, stop early
            if (!spindexer.hasAnyBall()) break;

            // --- 2a) Send a single rising edge on "Y" to start this shot ---
            spindexer.update(telemetry, loader, true, 0);   // yEdge = true ONCE
            loader.updateLoader();
            shooter.update(true, false, false, fieldPos);

            telemetry.addData("Auto", "Shot %d: trigger", shot + 1);
            telemetry.update();
            idle();

            // --- 2b) Let this one shot run until spindexer finishes it ---
            long shotTimeout = System.currentTimeMillis() + 2000; // 2s safety timeout per shot
            while (opModeIsActive()
                    && System.currentTimeMillis() < shotTimeout) {

                // No new Y-edge while shot is running
                spindexer.update(telemetry, loader, false, 0);
                loader.updateLoader();
                shooter.update(true, false, false, fieldPos);

                telemetry.addData("Auto", "Shot %d running", shot + 1);
                telemetry.addData("Ejecting", spindexer.isEjecting());
                telemetry.addData("HasAnyBall", spindexer.hasAnyBall());
                telemetry.update();

                // If your spindexer has an 'isEjecting' flag,
                // we can use the falling edge of that as "this shot is done"
                if (!spindexer.isEjecting()) {
                    break;
                }

                idle();
            }
        }

        // ===== 3) Brief spin-down window (optional) =====
        long holdEnd = System.currentTimeMillis() + 300; // extra 0.3s spin
        while (opModeIsActive() && System.currentTimeMillis() < holdEnd) {
            shooter.update(true, false, false, fieldPos);        // keep on briefly
            spindexer.update(telemetry, loader, false, 0);       // no new eject
            loader.updateLoader();

            telemetry.addData("Auto", "Post-shoot hold");
            telemetry.update();
            idle();
        }

        // Finally turn shooter off hard
        shooter.update(false, false, false, fieldPos);
        shooter.stop();    // ensure motor power = 0
    }



    /**
     * Let shooter run for a bit, then toggle it off.
     */
    private void spinDownShooterGracefully(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            // Keep shooter on during the spin-down window
            ensureShooterOn();
            loader.updateLoader();
            spindexer.update(telemetry, loader, false, 0);

            telemetry.addData("Phase", "Shooter spin-down");
            telemetry.update();
            idle();
        }
        // Now turn shooter off
        ensureShooterOff();
    }

    // ---------------- INTAKE + SPINDEXER LOOP ----------------

    /**
     * Turn on intake and let spindexer auto-index while robot moves.
     * You plug your drive code into the loop where indicated.
     */
    private void runIntakeAndSpindexerLoop() {
        long start = System.currentTimeMillis();

        // Intake on for the whole phase
        intake.startIntake();

        // Make sure shooter is off at start of this phase
        shooter.stop();
        shooter.update(false, false, false, fieldPos);

        while (opModeIsActive() && System.currentTimeMillis() - start < INTAKE_PHASE_MS) {
            // ----- YOUR DRIVE PATH HERE -----
            drive.drive(0.0, 0.0, 0.0);  // placeholder

            // Keep shooter off
            shooter.update(false, false, false, fieldPos);

            // Let loader run its state machine (usually idle unless triggered)
            loader.updateLoader();

            // Let spindexer auto-intake and rotate as normal
            // yEdge = false (we're not shooting), patternTagOverride = 0 (fast mode)
            spindexer.update(telemetry, loader, false, 0);

            telemetry.addData("Phase", "Intake + Spindexer loop");
            telemetry.addData("Spd intakeSlot", spindexer.getIntakeSlotIndex());
            telemetry.addData("Spd full", spindexer.isFull());
            telemetry.update();

            idle();
        }

        // Stop drive and intake at end of phase
        drive.stopAll();
        intake.stopIntake();
    }


    /**
     * Spins up the shooter and fires all balls currently in the spindexer
     * using the existing fastest-possible pattern (tag = 0).
     *
     * - Keeps shooter at speed across all shots
     * - Uses your existing spindexer.update(...) state machine
     */
    private void autoShootAllFastest(DrivetrainSubsystem drive,
                                     ShooterSubsystem shooter,
                                     SpindexerSubsystem spindexer,
                                     LoaderSubsystem loader,
                                     int fieldPos) {

        // ===== 1) Spin up shooter once =====

        // We want to simulate ONE rising edge on the toggle button
        boolean toggleButton = !shooter.isOn();  // if it's off, we need one press

        long spinupEnd = System.currentTimeMillis() + 800; // 0.8s spin-up, tune as needed

        while (opModeIsActive() && System.currentTimeMillis() < spinupEnd) {
            // First loop: toggleButton may be true to turn shooter on
            shooter.update(toggleButton, false, false, fieldPos);
            toggleButton = false;  // after first call, never toggle again

            // Keep spindexer/loader logic alive (no eject yet)
            spindexer.update(telemetry, loader, false, 0);
            loader.updateLoader();

            telemetry.addData("Auto", "Spinning up shooter...");
            telemetry.addData("Shooter RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
        }

        // ===== 2) Start spindexer eject sequence (fastest order, tag=0) =====

        boolean startedEject = false;

        while (opModeIsActive()) {
            // On the first loop we send yEdge = true to start the eject sequence.
            boolean yEdge = !startedEject;

            // Pattern override = 0 -> "fastest possible"
            spindexer.update(telemetry, loader, yEdge, 0);
            loader.updateLoader();

            // Keep shooter running at current RPM the whole time (no further toggles)
            shooter.update(false, false, false, fieldPos);

            telemetry.addData("Auto", "Ejecting balls...");
            telemetry.addData("Spd ejecting", spindexer.isEjecting());
            telemetry.addData("Spd hasAnyBall", spindexer.hasAnyBall());
            telemetry.update();

            startedEject = true;

            // Done when spindexer is not ejecting AND no balls left
            if (!spindexer.isEjecting() && !spindexer.hasAnyBall()) {
                break;
            }
        }

        // ===== 3) Turn shooter off at the end (optional spin-down grace) =====

        // If you want a 0.5s spin-down delay, you can sleep here:
        // sleep(500);

        shooter.stop();
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_new;

@TeleOp(name="TUNE Spindexer PIDF (Panels)_State_new", group="Tuning")
public class SpindexerPIDFTunerTeleOp_State_new extends OpMode {

    private SpindexerSubsystem_State_new spindexer;

    // Panels telemetry handle (wrapper around Panels’ Telemetry)
    private Telemetry panelsTelemetry;

    private boolean lastY = false;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystem_State_new(hardwareMap);

        // === PANELS TELEMETRY SETUP (same pattern as your working OpMode) ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();

        telemetry.addLine("Spindexer PIDF Tuner (no loader) Ready");
        telemetry.addLine("Y: trigger eject sequence (spindexer only)");
        telemetry.addLine("Tune kP/kI/kD/kF in Panels: SpindexerTuningConfig_new");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Rising edge of Y to start eject sequence if you want to test it
        boolean yEdge = gamepad1.y && !lastY;
        lastY = gamepad1.y;

        // Tag override can still come from Panels config
        int tagOverride = SpindexerTuningConfig_new.patternTagOverride;

        // Run spindexer state machine + PIDF.
        // Pass loader = null so only the spindexer spins.
        spindexer.update(panelsTelemetry, null, yEdge, tagOverride);

        // ====== Driver Station telemetry (quick view) ======
        telemetry.addData("AngleCurr", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("AngleTarget", "%.1f", spindexer.getTargetAngleDeg());
        telemetry.addData("Angle Error", "%.1f", (spindexer.getTargetAngleDeg()- spindexer.getCurrentAngleDeg()));
        telemetry.addData("IntakeIndex", spindexer.getIntakeIndex());
        telemetry.addData("HasAnyBall", spindexer.hasAnyBall());
        telemetry.addData("IsFull", spindexer.isFull());
        telemetry.addData("IsEjecting", spindexer.isEjecting());

        telemetry.addData("kP", SpindexerTuningConfig_new.kP);
        telemetry.addData("kI", SpindexerTuningConfig_new.kI);
        telemetry.addData("kD", SpindexerTuningConfig_new.kD);
        telemetry.addData("kF", SpindexerTuningConfig_new.kF);
        telemetry.update();

        // ====== PANELS telemetry (for graphs) ======
        panelsTelemetry.addData("sp_angle_curr", "%.1f", spindexer.getCurrentAngleDeg());
        panelsTelemetry.addData("sp_angle_tgt", "%.1f", spindexer.getTargetAngleDeg());
        panelsTelemetry.addData("ap_angle_err", "%.1f", (spindexer.getTargetAngleDeg()- spindexer.getCurrentAngleDeg()));
        panelsTelemetry.addData("sp_intake_idx", spindexer.getIntakeIndex());
        panelsTelemetry.addData("sp_has_ball", spindexer.hasAnyBall() ? 1 : 0);
        panelsTelemetry.addData("sp_is_full",  spindexer.isFull() ? 1 : 0);
        panelsTelemetry.addData("sp_is_ejecting", spindexer.isEjecting() ? 1 : 0);

        panelsTelemetry.addData("sp_kP", SpindexerTuningConfig_new.kP);
        panelsTelemetry.addData("sp_kI", SpindexerTuningConfig_new.kI);
        panelsTelemetry.addData("sp_kD", SpindexerTuningConfig_new.kD);
        panelsTelemetry.addData("sp_kF", SpindexerTuningConfig_new.kF);

        panelsTelemetry.update();
    }
}

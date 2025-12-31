package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_State;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

@TeleOp(name="TUNE Spindexer PIDF (Panels) _State", group="Tuning")
public class SpindexerPIDFTunerTeleOp_State extends OpMode {

    private SpindexerSubsystem_State spindexer;
    private IntakeSubsystem_Motor intake;
    private Telemetry panelsTelemetry;   // <- Panels telemetry handle

    private int selectedSlot = 0;

    private boolean prevA, prevB, prevX, prevY, prevDpadL, prevDpadR;

    private enum CycleState { IDLE, MOVE_INTAKE, WAIT_SETTLE, MOVE_LOAD, WAIT_SETTLE_2 }
    private CycleState cycleState = CycleState.IDLE;
    private long stateDeadlineMs = 0;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystem_State(hardwareMap);
        intake = new IntakeSubsystem_Motor(hardwareMap);
        intake.startIntake();;

        // === PANELS TELEMETRY SETUP ===
        //panelsTelemetry = PanelsTelemetry.INSTANCE.telemetry;                 // Panels 1.0 style (@JvmField)
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();

        telemetry.addLine("Spindexer PIDF Tuner Ready");
        telemetry.addLine("DPAD L/R: select slot (0/1/2)");
        telemetry.addLine("A: move selected slot to INTAKE");
        telemetry.addLine("B: move selected slot to LOAD");
        telemetry.addLine("X: home (slot 0 intake)");
        telemetry.addLine("Y: toggle AUTO cycle");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // Keep subsystem alive (PIDF apply, non-blocking motion, active hold)
        spindexer.periodic();

        boolean aEdge = gamepad1.a && !prevA;
        boolean bEdge = gamepad1.b && !prevB;
        boolean xEdge = gamepad1.x && !prevX;
        boolean yEdge = gamepad1.y && !prevY;
        boolean leftEdge  = gamepad1.dpad_left  && !prevDpadL;
        boolean rightEdge = gamepad1.dpad_right && !prevDpadR;

        prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x; prevY = gamepad1.y;
        prevDpadL = gamepad1.dpad_left; prevDpadR = gamepad1.dpad_right;

        if (leftEdge)  selectedSlot = (selectedSlot + 2) % 3;
        if (rightEdge) selectedSlot = (selectedSlot + 1) % 3;

        if (aEdge) {
            cycleState = CycleState.IDLE;
            SpindexerTuningConfig_State.AUTO_CYCLE = false;
            spindexer.cmdIntakeSlot(selectedSlot);
        }

        if (bEdge) {
            cycleState = CycleState.IDLE;
            SpindexerTuningConfig_State.AUTO_CYCLE = false;
            spindexer.cmdLoadSlot(selectedSlot);
        }

        if (xEdge) {
            cycleState = CycleState.IDLE;
            SpindexerTuningConfig_State.AUTO_CYCLE = false;
            spindexer.cmdIntakeSlot(0);
        }

        if (yEdge) {
            SpindexerTuningConfig_State.AUTO_CYCLE = !SpindexerTuningConfig_State.AUTO_CYCLE;
            cycleState = SpindexerTuningConfig_State.AUTO_CYCLE ? CycleState.MOVE_INTAKE : CycleState.IDLE;
        }

        if (SpindexerTuningConfig_State.AUTO_CYCLE) {
            switch (cycleState) {
                case MOVE_INTAKE:
                    if (!spindexer.isMoving()) {
                        spindexer.cmdIntakeSlot(selectedSlot);
                        cycleState = CycleState.WAIT_SETTLE;
                        stateDeadlineMs = now + SpindexerTuningConfig_State.SETTLE_MS;
                    }
                    break;

                case WAIT_SETTLE:
                    if (!spindexer.isMoving() && now >= stateDeadlineMs) {
                        spindexer.cmdLoadSlot(selectedSlot);
                        cycleState = CycleState.WAIT_SETTLE_2;
                        stateDeadlineMs = now + SpindexerTuningConfig_State.SETTLE_MS;
                    }
                    break;

                case WAIT_SETTLE_2:
                    if (!spindexer.isMoving() && now >= stateDeadlineMs) {
                        selectedSlot = (selectedSlot + 1) % 3;
                        cycleState = CycleState.MOVE_INTAKE;
                    }
                    break;

                default:
                case IDLE:
                    cycleState = CycleState.MOVE_INTAKE;
                    break;
            }
        }

        int pos = spindexer.getEncoder();
        int tgt = spindexer.getTarget();
        int err = tgt - pos;

        telemetry.addData("Selected Slot", selectedSlot);
        telemetry.addData("Auto Cycle", SpindexerTuningConfig_State.AUTO_CYCLE);
        telemetry.addData("Moving", spindexer.isMoving());

        telemetry.addData("sp_pos", pos);
        telemetry.addData("sp_tgt", tgt);
        telemetry.addData("sp_err", err);
        telemetry.addData("sp_angle", spindexer.getCurrentAngleDeg());

        telemetry.addData("POS_P", SpindexerTuningConfig_State.POS_P);
        telemetry.addData("VEL_P/I/D/F", "%.2f %.2f %.2f %.2f",
                SpindexerTuningConfig_State.VEL_P,
                SpindexerTuningConfig_State.VEL_I,
                SpindexerTuningConfig_State.VEL_D,
                SpindexerTuningConfig_State.VEL_F
        );

        telemetry.addData("HOLD", "%s pwr=%.2f db=%d",
                SpindexerTuningConfig_State.HOLD_ENABLED,
                SpindexerTuningConfig_State.HOLD_POWER,
                SpindexerTuningConfig_State.HOLD_DEADBAND_TICKS
        );

        telemetry.update();

        // ========= PANELS GRAPH FEED =========
        // Panels Graph looks for `<name>:<number>` in telemetry:
        // addData("sp_err", err) -> "sp_err: 123"
        panelsTelemetry.addData("sp_pos", pos);
        panelsTelemetry.addData("sp_tgt", tgt);
        panelsTelemetry.addData("sp_err", err);
        panelsTelemetry.addData("sp_angle", spindexer.getCurrentAngleDeg());

        // You can also throw in your tuning vars if you want them on the graph:
        panelsTelemetry.addData("sp_POS_P", SpindexerTuningConfig_State.POS_P);
        panelsTelemetry.addData("sp_HOLD_POWER", SpindexerTuningConfig_State.HOLD_POWER);

        panelsTelemetry.update();
    }
}

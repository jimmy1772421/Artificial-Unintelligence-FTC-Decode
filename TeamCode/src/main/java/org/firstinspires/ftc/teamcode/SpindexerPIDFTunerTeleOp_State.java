package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_State;


@TeleOp(name="TUNE Spindexer PIDF (Panels)", group="Tuning")
public class SpindexerPIDFTunerTeleOp_State extends OpMode {

    private SpindexerSubsystem_State spindexer;

    private int selectedSlot = 0;

    // Edge detection
    private boolean prevA, prevB, prevX, prevY, prevDpadL, prevDpadR;

    // Auto-cycle little state machine
    private enum CycleState { IDLE, MOVE_INTAKE, WAIT_SETTLE, MOVE_LOAD, WAIT_SETTLE_2 }
    private CycleState cycleState = CycleState.IDLE;
    private long stateDeadlineMs = 0;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystem_State(hardwareMap);

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

        // Always run the spindexer's periodic (this is what makes it non-blocking)
        spindexer.periodic();

        // ---- Button edges ----
        boolean aEdge = gamepad1.a && !prevA;
        boolean bEdge = gamepad1.b && !prevB;
        boolean xEdge = gamepad1.x && !prevX;
        boolean yEdge = gamepad1.y && !prevY;
        boolean leftEdge  = gamepad1.dpad_left  && !prevDpadL;
        boolean rightEdge = gamepad1.dpad_right && !prevDpadR;

        prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x; prevY = gamepad1.y;
        prevDpadL = gamepad1.dpad_left; prevDpadR = gamepad1.dpad_right;

        // ---- Slot selection ----
        if (leftEdge)  selectedSlot = (selectedSlot + 2) % 3; // -1 mod 3
        if (rightEdge) selectedSlot = (selectedSlot + 1) % 3;

        // ---- Manual commands (disable auto-cycle state machine if you take manual control) ----
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
            if (!SpindexerTuningConfig_State.AUTO_CYCLE) cycleState = CycleState.IDLE;
            else cycleState = CycleState.MOVE_INTAKE;
        }

        // ---- Auto-cycle (intake -> load -> next slot) ----
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
                        // advance to next slot
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

        // ---- Telemetry you want while tuning ----
        int pos = spindexer.getEncoder();
        int tgt = spindexer.getTarget();
        int err = tgt - pos;

        telemetry.addData("Selected Slot", selectedSlot);
        telemetry.addData("Auto Cycle", SpindexerTuningConfig_State.AUTO_CYCLE);
        telemetry.addData("Moving", spindexer.isMoving());

        telemetry.addData("pos/tgt/err", "%d / %d / %d", pos, tgt, err);
        telemetry.addData("angleDeg(enc)", "%.1f", spindexer.getCurrentAngleDeg());

        telemetry.addData("POS_P", SpindexerTuningConfig_State.POS_P);
        telemetry.addData("VEL_P/I/D/F", "%.2f %.2f %.2f %.2f",
                SpindexerTuningConfig_State.VEL_P,
                SpindexerTuningConfig_State.VEL_I,
                SpindexerTuningConfig_State.VEL_D,
                SpindexerTuningConfig_State.VEL_F
        );

        telemetry.update();
    }
}

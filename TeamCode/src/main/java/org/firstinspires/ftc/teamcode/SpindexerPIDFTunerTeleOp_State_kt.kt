package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_State

@TeleOp(
    name = "TUNE Spindexer PIDF (Panels) _State_kt",
    group = "Tuning"
)
class SpindexerPIDFTunerTeleOp_State_kt : OpMode() {

    private lateinit var spindexer: SpindexerSubsystem_State

    // Panels telemetry – same style as your TestGraph example
    private val panelsTelemetry = PanelsTelemetry.telemetry

    private var selectedSlot = 0

    private var prevA = false
    private var prevB = false
    private var prevX = false
    private var prevY = false
    private var prevDpadL = false
    private var prevDpadR = false

    private enum class CycleState {
        IDLE,
        MOVE_INTAKE,
        WAIT_SETTLE,
        MOVE_LOAD,
        WAIT_SETTLE_2
    }

    private var cycleState = CycleState.IDLE
    private var stateDeadlineMs = 0L

    override fun init() {
        spindexer = SpindexerSubsystem_State(hardwareMap)

        telemetry.addLine("Spindexer PIDF Tuner Ready")
        telemetry.addLine("DPAD L/R: select slot (0/1/2)")
        telemetry.addLine("A: move selected slot to INTAKE")
        telemetry.addLine("B: move selected slot to LOAD")
        telemetry.addLine("X: home (slot 0 intake)")
        telemetry.addLine("Y: toggle AUTO cycle")
        telemetry.update()
    }

    override fun loop() {
        val now = System.currentTimeMillis()

        // Keep subsystem alive (PIDF apply, non-blocking motion, active hold)
        spindexer.periodic()

        val aEdge = gamepad1.a && !prevA
        val bEdge = gamepad1.b && !prevB
        val xEdge = gamepad1.x && !prevX
        val yEdge = gamepad1.y && !prevY
        val leftEdge = gamepad1.dpad_left && !prevDpadL
        val rightEdge = gamepad1.dpad_right && !prevDpadR

        prevA = gamepad1.a
        prevB = gamepad1.b
        prevX = gamepad1.x
        prevY = gamepad1.y
        prevDpadL = gamepad1.dpad_left
        prevDpadR = gamepad1.dpad_right

        if (leftEdge)  selectedSlot = (selectedSlot + 2) % 3
        if (rightEdge) selectedSlot = (selectedSlot + 1) % 3

        if (aEdge) {
            cycleState = CycleState.IDLE
            SpindexerTuningConfig_State.AUTO_CYCLE = false
            spindexer.cmdIntakeSlot(selectedSlot)
        }

        if (bEdge) {
            cycleState = CycleState.IDLE
            SpindexerTuningConfig_State.AUTO_CYCLE = false
            spindexer.cmdLoadSlot(selectedSlot)
        }

        if (xEdge) {
            cycleState = CycleState.IDLE
            SpindexerTuningConfig_State.AUTO_CYCLE = false
            spindexer.cmdIntakeSlot(0)
        }

        if (yEdge) {
            SpindexerTuningConfig_State.AUTO_CYCLE = !SpindexerTuningConfig_State.AUTO_CYCLE
            cycleState = if (SpindexerTuningConfig_State.AUTO_CYCLE) {
                CycleState.MOVE_INTAKE
            } else {
                CycleState.IDLE
            }
        }

        if (SpindexerTuningConfig_State.AUTO_CYCLE) {
            when (cycleState) {
                CycleState.MOVE_INTAKE -> {
                    if (!spindexer.isMoving()) {
                        spindexer.cmdIntakeSlot(selectedSlot)
                        cycleState = CycleState.WAIT_SETTLE
                        stateDeadlineMs = now + SpindexerTuningConfig_State.SETTLE_MS
                    }
                }

                CycleState.WAIT_SETTLE -> {
                    if (!spindexer.isMoving() && now >= stateDeadlineMs) {
                        spindexer.cmdLoadSlot(selectedSlot)
                        cycleState = CycleState.WAIT_SETTLE_2
                        stateDeadlineMs = now + SpindexerTuningConfig_State.SETTLE_MS
                    }
                }

                // not actually used, but included so the when is fully exhaustive
                CycleState.MOVE_LOAD -> {
                    // no-op for now
                }

                CycleState.WAIT_SETTLE_2 -> {
                    if (!spindexer.isMoving() && now >= stateDeadlineMs) {
                        selectedSlot = (selectedSlot + 1) % 3
                        cycleState = CycleState.MOVE_INTAKE
                    }
                }

                CycleState.IDLE -> {
                    // if AUTO_CYCLE somehow starts in IDLE, kick it into MOVE_INTAKE
                    cycleState = CycleState.MOVE_INTAKE
                }
            }
        }

        val pos = spindexer.encoder
        val tgt = spindexer.target
        val err = tgt - pos
        val angleDeg = spindexer.currentAngleDeg

        // ===== Driver Station / DS telemetry =====
        telemetry.addData("Selected Slot", selectedSlot)
        telemetry.addData("Auto Cycle", SpindexerTuningConfig_State.AUTO_CYCLE)
        telemetry.addData("Moving", spindexer.isMoving())

        telemetry.addData("sp_pos", pos)
        telemetry.addData("sp_tgt", tgt)
        telemetry.addData("sp_err", err)
        telemetry.addData("sp_angle", angleDeg)

        telemetry.addData("POS_P", SpindexerTuningConfig_State.POS_P)
        telemetry.addData(
            "VEL_P/I/D/F", "%.2f %.2f %.2f %.2f",
            SpindexerTuningConfig_State.VEL_P,
            SpindexerTuningConfig_State.VEL_I,
            SpindexerTuningConfig_State.VEL_D,
            SpindexerTuningConfig_State.VEL_F
        )

        telemetry.addData(
            "HOLD", "%s pwr=%.2f db=%d",
            SpindexerTuningConfig_State.HOLD_ENABLED,
            SpindexerTuningConfig_State.HOLD_POWER,
            SpindexerTuningConfig_State.HOLD_DEADBAND_TICKS
        )

        telemetry.update()

        // ===== Panels Graph feed (Panels reads `<name>:<number>` from telemetry) =====
        panelsTelemetry.addData("sp_pos", pos)
        panelsTelemetry.addData("sp_tgt", tgt)
        panelsTelemetry.addData("sp_err", err)
        panelsTelemetry.addData("sp_angle", angleDeg)

        // Optional: also stream tunables so you can see when you change them
        panelsTelemetry.addData("sp_POS_P", SpindexerTuningConfig_State.POS_P)
        panelsTelemetry.addData("sp_HOLD_POWER", SpindexerTuningConfig_State.HOLD_POWER)

        panelsTelemetry.update(telemetry)
    }
}

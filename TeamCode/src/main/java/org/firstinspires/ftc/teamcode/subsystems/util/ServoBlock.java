package org.firstinspires.ftc.teamcode.subsystems.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Treats two CRServos + an absolute encoder as a single "motor-like" unit.
 *
 * Supports:
 * - setMode(RUN_USING_ENCODER / RUN_WITHOUT_ENCODER / RUN_TO_POSITION)
 * - setTargetPosition(ticks)
 * - setPower(...)
 * - getCurrentPosition()
 * - setDirection(...)
 * - setZeroPowerBehavior(...) (best-effort; for CR servos it's mostly "hold via controller")
 *
 * IMPORTANT: call update() every loop for RUN_TO_POSITION to work.
 */
@Configurable
public class ServoBlock {

    // --- RUN_TO_POSITION controller tuning (start conservative) ---
    public static double POS_KP = 0.00006;         // power per tick error
    public static int POS_DEADBAND_TICKS = 25;     // within this, output = 0
    public static double POS_MIN_POWER = 0.06;     // optional: overcome friction (0..1). Keep 0 if you don't want it.

    // D term (damping)
    public static double POS_KD = 0.00000008;   // power per (tick/sec)

    // Slow near target
    public static int SLOW_ZONE_TICKS = 600; // start slowing inside this error
    public static double SLOW_MAX_POWER = 0.08;

    // Output slew limit (prevents instant reversals)
    public static double CMD_SLEW_PER_SEC = 4.0;

    private int lastError = 0;
    private long lastTimeNanos = 0;
    private double lastCmd = 0.0;
    public static double MIN_EFFECTIVE_POWER = 0.071;   // your measured threshold
    public static int STOP_TICKS = 40;                 // final hold window (tune)

    public static int HOLD_TICKS = 40;        // inside this, stop output
    public static int REENGAGE_TICKS = 120;   // if pushed beyond this, fight back again


    private final CRServo servoA;
    private final CRServo servoB;
    private final WrappedAnalogAbsEncoder encoder;

    private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    private double openLoopPower = 0.0;

    private int targetPositionTicks = 0;
    private double runToPosMaxPower = 0.0; // magnitude limit for RUN_TO_POSITION
    private int dirSign = 1;
    private boolean holding = false;

    public ServoBlock(HardwareMap hw,
                      String servoAName,
                      String servoBName,
                      String absAnalogName) {

        servoA = hw.get(CRServo.class, servoAName);
        servoB = hw.get(CRServo.class, servoBName);

        AnalogInput abs = hw.get(AnalogInput.class, absAnalogName);
        encoder = new WrappedAnalogAbsEncoder(abs);
        encoder.resetTrackingHere();

        // You said both servos face/turn the same direction:
        servoA.setDirection(DcMotorSimple.Direction.FORWARD);
        servoB.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // -------------------------
    // Motor-like API
    // -------------------------

    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
        // DO NOT change targetPositionTicks here.
    }

    public DcMotor.RunMode getMode() {
        return mode;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.zeroPowerBehavior = behavior;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
        dirSign = (direction == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
    }

    public int getCurrentPosition() {
        return dirSign * encoder.getCurrentTicks();
    }

    public void setTargetPosition(int ticks) {
        this.targetPositionTicks = ticks;
    }

    public int getTargetPosition() {
        return targetPositionTicks;
    }

    /**
     * In RUN_TO_POSITION: power acts like "max power" (magnitude limit), like a motor.
     * In other modes: power is direct open-loop command.
     */
    public void setPower(double power) {
        power = Range.clip(power, -1.0, 1.0);

        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            runToPosMaxPower = Math.abs(power);
            // don't directly write to servos here; update() does closed-loop
            return;
        }

        openLoopPower = power;
        applyServoPower(openLoopPower);
    }

    public double getPower() {
        return (mode == DcMotor.RunMode.RUN_TO_POSITION) ? runToPosMaxPower : openLoopPower;
    }

    /** Call every loop. Required for RUN_TO_POSITION. */
    public void update() {
        if (mode != DcMotor.RunMode.RUN_TO_POSITION) return;

        long now = System.nanoTime();
        double dt = (lastTimeNanos == 0) ? 0.02 : (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;
        if (dt < 1e-4) dt = 1e-4;

        int current = getCurrentPosition();
        int error = targetPositionTicks - current;

        // Hold hysteresis: don't buzz at target, but re-engage if pushed away
        if (holding) {
            if (Math.abs(error) >= REENGAGE_TICKS) holding = false;
            else {
                applyServoPower(0.0);
                lastCmd = 0.0;
                lastError = error;
                return;
            }
        } else {
            if (Math.abs(error) <= HOLD_TICKS) {
                holding = true;
                applyServoPower(0.0);
                lastCmd = 0.0;
                lastError = error;
                return;
            }
        }

        // derivative
        double errorRate = (error - lastError) / dt;
        lastError = error;

        // max power logic (never below minimum effective)
        double maxP = runToPosMaxPower;
        if (Math.abs(error) < SLOW_ZONE_TICKS) maxP = Math.min(maxP, SLOW_MAX_POWER);
        maxP = Math.max(maxP, MIN_EFFECTIVE_POWER);

        // PD
        double cmd = POS_KP * error + POS_KD * errorRate;

        // clip
        cmd = Range.clip(cmd, -maxP, maxP);

        // slew limit
        double maxDelta = CMD_SLEW_PER_SEC * dt;
        cmd = Range.clip(cmd, lastCmd - maxDelta, lastCmd + maxDelta);

        // UNIVERSAL minimum: if correcting, always at least MIN_EFFECTIVE_POWER
        if (Math.abs(cmd) < MIN_EFFECTIVE_POWER) {
            cmd = Math.signum(error) * MIN_EFFECTIVE_POWER;
        }

        // final safety clip
        cmd = Range.clip(cmd, -maxP, maxP);

        lastCmd = cmd;
        applyServoPower(cmd);
    }



    // -------------------------
    // Internals
    // -------------------------


    private void applyServoPower(double pwr) {
        double out = Range.clip(pwr, -1.0, 1.0);
        out *= dirSign;
        servoA.setPower(out);
        servoB.setPower(out);
    }
}

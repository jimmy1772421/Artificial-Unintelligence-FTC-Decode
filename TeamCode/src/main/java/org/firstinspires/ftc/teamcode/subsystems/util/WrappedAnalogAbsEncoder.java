package org.firstinspires.ftc.teamcode.subsystems.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Wraps a 0..maxV analog absolute encoder into a continuous (incremental-like) position.
 * Output is "ticks" where TICKS_PER_REV corresponds to 1 mechanical revolution of the encoder shaft.
 */
@Configurable
public class WrappedAnalogAbsEncoder {

    // Treat one full revolution as this many "ticks" (pick something reasonable & consistent).
    // 4096 is a nice default (12-bit-ish scale).
    public static int TICKS_PER_REV = 4096;

    // Offset applied to the normalized [0,1) reading (lets you align electrical zero to your chosen zero).
    // Units: normalized turns (0.0..1.0). Example: 0.25 = +90deg shift.
    public static double OFFSET_TURNS = 0.0;

    // If true, invert direction (increasing angle becomes decreasing).
    public static boolean REVERSED = false;

    // Wrap detection threshold (0.5 means any jump > 180deg counts as wrap).
    public static double WRAP_THRESHOLD_TURNS = 0.5;

    private final AnalogInput abs;

    private double lastTurns0to1 = Double.NaN;
    private long wrapCount = 0; // counts full revolutions crossed

    public WrappedAnalogAbsEncoder(AnalogInput abs) {
        this.abs = abs;
    }

    /** Call once at init so the first delta doesn't accidentally wrap. */
    public void resetTrackingHere() {
        lastTurns0to1 = readTurns0to1();
        wrapCount = 0;
    }

    /** Normalized absolute position in [0,1). Includes OFFSET_TURNS and REVERSED. */
    public double readTurns0to1() {
        double v = abs.getVoltage();
        double maxV = abs.getMaxVoltage(); // should be ~3.3 on Control/Expansion Hub

        double turns = (maxV <= 1e-6) ? 0.0 : (v / maxV);
        // clamp to safe range then wrap to [0,1)
        turns = turns - Math.floor(turns);

        // apply offset
        turns = turns + OFFSET_TURNS;
        turns = turns - Math.floor(turns);

        if (REVERSED) {
            turns = 1.0 - turns;
            // keep in [0,1)
            turns = turns - Math.floor(turns);
        }

        return turns;
    }

    /** Continuous turns since resetTrackingHere() (can go negative). */
    public double getContinuousTurns() {
        double now = readTurns0to1();

        if (Double.isNaN(lastTurns0to1)) {
            lastTurns0to1 = now;
            wrapCount = 0;
            return wrapCount + now;
        }

        double delta = now - lastTurns0to1;

        // crossed wrap boundary?
        if (delta > WRAP_THRESHOLD_TURNS) {
            // e.g. 0.02 -> 0.98 when moving negative
            wrapCount--;
        } else if (delta < -WRAP_THRESHOLD_TURNS) {
            // e.g. 0.98 -> 0.02 when moving positive
            wrapCount++;
        }

        lastTurns0to1 = now;
        return wrapCount + now;
    }

    /** Continuous ticks since resetTrackingHere(). This emulates an incremental encoder. */
    public int getCurrentTicks() {
        double contTurns = getContinuousTurns();
        long ticks = Math.round(contTurns * (long) TICKS_PER_REV);
        // motor encoders return int
        if (ticks > Integer.MAX_VALUE) return Integer.MAX_VALUE;
        if (ticks < Integer.MIN_VALUE) return Integer.MIN_VALUE;
        return (int) ticks;
    }
}

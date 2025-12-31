package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerTuningConfig_State {

    // ----- Tuner behavior -----
    public static boolean AUTO_CYCLE = false;
    public static long SETTLE_MS = 150;

    // ----- Motion -----
    public static double MOVE_POWER = 0.50;
    public static long MOVE_TIMEOUT_MS = 1500;

    // ----- RUN_TO_POSITION Position PIDF -----
    // These control how aggressively the motor goes to a target position
    public static double POS_P = 5.0;
    public static double POS_I = 0.0;
    public static double POS_D = 0.0;
    // Usually 0 for position, but exposed in case you want it
    public static double POS_F = 0.0;

    // ----- RUN_USING_ENCODER Velocity PIDF (optional; mostly used for flywheels) -----
    public static double VEL_P = 5.0;
    public static double VEL_I = 0.0;
    public static double VEL_D = 0.0;
    public static double VEL_F = 11.8; // good starting point for goBILDA 435rpm (ticks/s)

    // ----- Active hold (keeps position corrected while idle) -----
    public static boolean HOLD_ENABLED = true;
    public static double HOLD_POWER = 0.12;
    public static int HOLD_DEADBAND_TICKS = 6;
}

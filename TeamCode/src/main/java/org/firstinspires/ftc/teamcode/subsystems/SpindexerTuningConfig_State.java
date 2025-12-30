package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerTuningConfig_State {
    // ===== Position loop (RUN_TO_POSITION): treat this as "Position P"
    public static double POS_P = 6.0;

    // ===== Velocity loop (RUN_USING_ENCODER): tune this for smooth motion
    public static double VEL_P = 10.0;
    public static double VEL_I = 0.0;
    public static double VEL_D = 0.0;
    public static double VEL_F = 0.0;

    // ===== Motion test parameters
    public static double MOVE_POWER = 0.6;
    public static long MOVE_TIMEOUT_MS = 900;

    // Auto cycle
    public static boolean AUTO_CYCLE = false;
    public static long SETTLE_MS = 150;
}

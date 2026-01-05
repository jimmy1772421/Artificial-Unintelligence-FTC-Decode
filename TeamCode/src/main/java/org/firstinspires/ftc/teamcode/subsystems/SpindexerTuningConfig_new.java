package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerTuningConfig_new {

    // PIDF gains for the spindexer angle controller
    public static double kP = 0.01;
    public static double kI = 0.0000;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Optional: override pattern tag from Panels (0, 21, 22, 23)
    public static int patternTagOverride = 0;
}

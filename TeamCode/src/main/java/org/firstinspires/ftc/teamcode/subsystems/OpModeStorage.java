package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Static memory that survives OpMode swaps (Auto -> TeleOp) as long as the RC app
 * is not restarted and the robot is not power-cycled.
 *
 * IMPORTANT:
 *  - This does NOT survive a REV Hub reboot / power cycle.
 *  - This WILL break if you call STOP_AND_RESET_ENCODER in a later OpMode.
 */
public final class OpModeStorage {
    private OpModeStorage() {}

    // ===== Turret incremental reference =====
    public static Double turretAngleOffsetDeg = null; // currentAngleDeg = offset + ticks/ticksPerDeg
    public static Boolean turretTrackEnabled = null;
    public static Double turretShaftContinuousDeg = null;

    // ===== Spindexer incremental reference =====
    public static Integer spindexerZeroTicks = null;   // ticks corresponding to internal angle 0 (slot0@intake = 0°)
    public static Integer spindexerIntakeIndex = null; // which slot index your code believes is at intake
    public static Double  spindexerTargetAngleDeg = null;

    // 0=EMPTY, 1=GREEN, 2=PURPLE
    public static int[] spindexerSlots = null;
    public static boolean turretInitDone = false;
}

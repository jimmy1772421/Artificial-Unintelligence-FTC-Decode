package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/** Shared state handoff Auto -> TeleOp (same RC app session). */
public final class PoseStorage {
    public static Pose lastPose = null;

    // Continuous turret heading in your turret frame (e.g. -200..+200-ish, can be continuous)
    public static Double lastTurretAngleDeg = null;

    private PoseStorage() {}
}

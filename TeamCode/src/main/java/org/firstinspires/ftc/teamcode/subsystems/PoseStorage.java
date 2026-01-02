package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/** Shared pose handoff Auto -> TeleOp (same RC app session). */
public final class PoseStorage {
    public static Pose lastPose = null;

    private PoseStorage() {}
}

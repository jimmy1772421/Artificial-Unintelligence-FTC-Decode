package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class VisionSubsystem {
    private final Limelight3A limelight3A;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        // Optional but nice
        limelight3A.setPollRateHz(100);

        // Your AprilTag pipeline index
        limelight3A.pipelineSwitch(8);

        limelight3A.start();
    }

    /**
     * Returns:
     *   21, 22, 23 = if one of those tag IDs is currently seen
     *   0          = if none of those tags are seen (no tags / other IDs / invalid)
     */
    public int getPattern() {
        LLResult result = limelight3A.getLatestResult();

        // No frame / invalid
        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return 0;
        }

        // Check all currently seen tags; return the first matching 21/22/23
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                return id;
            }
        }

        // None of the three we're interested in
        return 0;
    }
}

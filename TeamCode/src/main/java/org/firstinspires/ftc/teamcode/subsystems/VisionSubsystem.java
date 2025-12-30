package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Limelight AprilTag helper:
 * - Pattern tags: 21,22,23,24  (returns which one you see)
 * - Goal tags: 20 and 24       (returns tx (left/right) for turret aiming)
 *
 * IMPORTANT:
 * If your pipeline is configured with an ID filter that only includes 21-23,
 * you MUST update the Limelight pipeline settings to allow 20 and 24 too,
 * otherwise they will never appear in getFiducialResults().
 */
public class VisionSubsystem {

    private final Limelight3A limelight3A;

    // Your AprilTag pipeline index
    private static final int APRILTAG_PIPELINE = 0;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(APRILTAG_PIPELINE);
        limelight3A.start();
    }

    /** Small container for turret aiming. */
    public static class TagAim {
        public final int id;        // tag ID (20 or 24)
        public final double txDeg;  // left/right offset (deg). Typically: negative=left, positive=right
        public final double area;   // tag area (bigger = closer / more confident)

        public TagAim(int id, double txDeg, double area) {
            this.id = id;
            this.txDeg = txDeg;
            this.area = area;
        }
    }

    private LLResult getValidResultOrNull() {
        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result;
    }

    private List<LLResultTypes.FiducialResult> getTagsOrNull(LLResult result) {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;
        return tags;
    }

    /** Returns 21/22/23 if seen; else 0. Picks the largest-area one if multiple are visible. */
    public int getPatternTag() {
        LLResult result = getValidResultOrNull();
        List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
        if (tags == null) return 0;

        int bestId = 0;
        double bestArea = -1.0;

        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                double area = t.getTargetArea(); // % of image
                if (area > bestArea) {
                    bestArea = area;
                    bestId = id;
                }
            }
        }
        return bestId; // 0 if none of 21-24
    }

    /**
     * Best goal tag aim (20 or 24), chosen by larger area.
     * @return TagAim or null if no goal tag is seen.
     */
    public TagAim getBestGoalAim() {
        LLResult result = getValidResultOrNull();
        List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
        if (tags == null) return null;

        LLResultTypes.FiducialResult best = null;
        double bestArea = -1.0;

        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id == 20 || id == 24) {
                double area = t.getTargetArea();
                if (area > bestArea) {
                    bestArea = area;
                    best = t;
                }
            }
        }

        if (best == null) return null;

        return new TagAim(
                best.getFiducialId(),
                best.getTargetXDegrees(), // horizontal offset in degrees
                best.getTargetArea()
        );
    }

    /** Convenience: tx (deg) for the best goal tag (20/24). Returns NaN if none. */
    public double getGoalTxDegOrNaN() {
        TagAim aim = getBestGoalAim();
        return (aim == null) ? Double.NaN : aim.txDeg;
    }

    /** tx (deg) for a specific tag ID. Returns NaN if that tag isn't currently visible. */
    public double getTagTxDegOrNaN(int desiredId) {
        LLResult result = getValidResultOrNull();
        List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
        if (tags == null) return Double.NaN;

        for (LLResultTypes.FiducialResult t : tags) {
            if (t.getFiducialId() == desiredId) {
                return t.getTargetXDegrees();
            }
        }
        return Double.NaN;
    }

    /** Debug helper: returns comma-separated IDs currently visible ("" if none). */
    public String getSeenTagIdsString() {
        LLResult result = getValidResultOrNull();
        List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
        if (tags == null) return "";

        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult t : tags) {
            if (sb.length() > 0) sb.append(",");
            sb.append(t.getFiducialId());
        }
        return sb.toString();
    }
}

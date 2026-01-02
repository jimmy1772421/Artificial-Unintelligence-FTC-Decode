package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


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

    /**
     * Get a PedroPathing Pose from Limelight MegaTag2, using the current
     * Pedro follower pose to provide robot yaw (instead of raw IMU).
     *
     * @param desiredId  Tag ID to require (20 or 24). Pass <= 0 to accept any.
     * @param follower   Your Pedro Follower instance.
     * @return           Pedro Pose (PedroCoordinates) or null if no good vision.
     */
    public Pose getPedroPoseFromLimelight(int desiredId, Follower follower) {
        // ---- 0) Get latest Limelight result ----
        LLResult result = getValidResultOrNull();
        if (result == null) return null;

        // Optional: make sure we are actually seeing desiredId
        if (desiredId > 0) {
            List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
            boolean hasDesired = false;
            if (tags != null) {
                for (LLResultTypes.FiducialResult t : tags) {
                    if (t.getFiducialId() == desiredId) {
                        hasDesired = true;
                        break;
                    }
                }
            }
            if (!hasDesired) {
                return null; // not seeing the tag we care about
            }
        }

        // ---- 1) Get yaw from Pedro, convert to FTC field frame ----
        // follower.getPose() is in PedroCoordinates by default
        Pose pedroPoseNow = follower.getPose();

        // Convert that pose into FTC's coordinate system so heading matches the field map
        Pose ftcPoseNow = pedroPoseNow.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double robotYawRad = ftcPoseNow.getHeading();
        double robotYawDeg = Math.toDegrees(robotYawRad);

        // Tell Limelight our current orientation for MegaTag2
        limelight3A.updateRobotOrientation(robotYawDeg);

        // ---- 2) Read MegaTag2 pose from Limelight ----
        Pose3D botposeMt2 = result.getBotpose_MT2();
        if (botposeMt2 == null) {
            // Fallback: single-tag field-space pose for the desired tag
            List<LLResultTypes.FiducialResult> tags = getTagsOrNull(result);
            if (tags != null) {
                for (LLResultTypes.FiducialResult t : tags) {
                    if (desiredId <= 0 || t.getFiducialId() == desiredId) {
                        botposeMt2 = t.getRobotPoseFieldSpace();
                        break;
                    }
                }
            }
            if (botposeMt2 == null) return null;
        }

        // Basic quality gate: must see at least one tag for MT2
        if (result.getBotposeTagCount() == 0) {
            return null;
        }

        // Optional staleness gate
        if (result.getStaleness() > 50) { // > ~50ms old, tweak if needed
            return null;
        }

        // ---- 3) Extract FTC field pose (x, y, heading) from Pose3D ----
        double xField = botposeMt2.getPosition().x; // units match your fmap (Into The Deep = inches)
        double yField = botposeMt2.getPosition().y;
        double headingRad = botposeMt2.getOrientation().getYaw(AngleUnit.RADIANS);

        // Build pose in FTC coordinates
        Pose ftcPose = new Pose(xField, yField, headingRad, FTCCoordinates.INSTANCE);

        // Convert FTC pose into PedroCoordinates so you can feed it to Follower
        Pose pedroVisionPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        return pedroVisionPose;
    }

}

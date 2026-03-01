package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightSubSystem {

    private Limelight3A limelight;

    // ================= MOTIF TAG CONSTANTS =================
    // Tags 21, 22, 23 define the shooting pattern (ball color order).
    // They are NEVER used for navigation/botpose — filtered out below.
    private static final int MOTIF_TAG_MIN = 21;
    private static final int MOTIF_TAG_MAX = 23;

    // Ball color patterns per tag ID.
    // 1 = Green, 0 = Purple. Order = [shot1, shot2, shot3].
    // Tag 21 → GPP, Tag 22 → PGP, Tag 23 → PPG
    private static final int[][] MOTIF_PATTERNS = {
            {1, 0, 0},  // Tag 21: Green, Purple, Purple
            {0, 1, 0},  // Tag 22: Purple, Green, Purple
            {0, 0, 1}   // Tag 23: Purple, Purple, Green
    };

    // ================= APRIL TAG DATA CONTAINER =================
    /**
     * Holds all relevant data from a single limelight pipeline 0 read.
     *
     * tagVisible  → at least one NAVIGATION tag (not 21/22/23) was seen
     * poseValid   → botpose was successfully computed from navigation tags
     * tagId       → ID of the primary navigation tag seen
     * tx/ty/ta    → targeting angles and area for that primary tag
     * x/y/z       → robot field position (metres, from botpose)
     * yaw/pitch/roll → robot field orientation (degrees, from botpose)
     */
    public static class AprilTagData {
        public int    tagId    = -1;
        public String tagFamily = "";
        public double tx       = 0;
        public double ty       = 0;
        public double ta       = 0;

        // 3D robot field pose
        public double x     = 0;
        public double y     = 0;
        public double z     = 0;
        public double roll  = 0;
        public double pitch = 0;
        public double yaw   = 0;

        public boolean tagVisible = false; // navigation tag seen
        public boolean poseValid  = false; // botpose computed
    }

    // ================= CONSTRUCTOR =================
    public LimelightSubSystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    // ================================================================
    // METHOD: GET APRIL TAG DATA  (pipeline 0, navigation tags only)
    //
    // Returns targeting and botpose data from ALL fiducial tags EXCEPT
    // tags 21, 22, and 23, which are motif-only tags and must not
    // influence the robot's field position estimate.
    //
    // Returns a default (empty) AprilTagData object if no result is
    // available — never returns null, so callers don't need null checks.
    // ================================================================
    public AprilTagData getAprilTagData() {
        limelight.pipelineSwitch(0);

        LLResult result = limelight.getLatestResult();
        AprilTagData data = new AprilTagData();

        if (result == null) return data;

        // Pull the full fiducial list and filter out motif tags
        List<LLResultTypes.FiducialResult> allFiducials = result.getFiducialResults();
        if (allFiducials == null || allFiducials.isEmpty()) return data;

        // Build a filtered list containing only navigation tags
        LLResultTypes.FiducialResult primaryNavTag = null;
        for (LLResultTypes.FiducialResult f : allFiducials) {
            if (!isMotifTag(f.getFiducialId())) {
                // First non-motif tag found becomes the primary targeting reference
                if (primaryNavTag == null) {
                    primaryNavTag = f;
                }
            }
        }

        if (primaryNavTag != null) {
            data.tagVisible = true;
            data.tagId      = primaryNavTag.getFiducialId();
            data.tagFamily  = primaryNavTag.getFamily();
            data.tx         = primaryNavTag.getTargetXDegrees();
            data.ty         = primaryNavTag.getTargetYDegrees();
            data.ta         = primaryNavTag.getTargetArea();
        }

        // Botpose is computed by the limelight from all tags in view.
        // Because motif tags (21/22/23) are physical field markers at
        // known positions, having them in view still produces a valid
        // botpose — but to be safe and avoid any accidental influence
        // from misplaced motif tags, only trust botpose when at least
        // one proper navigation tag is visible.
        if (data.tagVisible) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                data.poseValid = true;
                data.x         = botpose.getPosition().x;
                data.y         = botpose.getPosition().y;
                data.z         = botpose.getPosition().z;
                data.yaw       = botpose.getOrientation().getYaw();
                data.pitch     = botpose.getOrientation().getPitch();
                data.roll      = botpose.getOrientation().getRoll();
            }
        }

        return data;
    }

    // ================================================================
    // METHOD: IDENTIFY MOTIF TAG
    //
    // Looks specifically for tags 21, 22, or 23 in the current frame.
    // On the FIRST successful detection, saves the corresponding ball
    // color pattern into MotifStorage so TeleOp can read it.
    //
    // Call this repeatedly in a loop (e.g. inside an Action) until
    // MotifStorage.motifDetected == true.
    //
    // Returns true  → motif tag was found and saved this call
    // Returns false → no motif tag visible this call (keep trying)
    //
    // Once MotifStorage.motifDetected is true, this method becomes a
    // no-op (returns true immediately) so it won't overwrite the saved
    // pattern if a different motif tag enters frame later.
    // ================================================================
    public boolean identifyMotifTag() {
        // Already locked in — don't overwrite
        if (MotifStorage.motifDetected) return true;

        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        if (result == null) return false;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return false;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (isMotifTag(id)) {
                // Map tag ID to pattern array index: 21→0, 22→1, 23→2
                int patternIndex = id - MOTIF_TAG_MIN;

                // Copy the pattern (don't store a reference to the constant array)
                int[] pattern = new int[3];
                pattern[0] = MOTIF_PATTERNS[patternIndex][0];
                pattern[1] = MOTIF_PATTERNS[patternIndex][1];
                pattern[2] = MOTIF_PATTERNS[patternIndex][2];

                // Save to static storage so TeleOp can access it
                MotifStorage.motifPattern  = pattern;
                MotifStorage.motifTagId    = id;
                MotifStorage.motifDetected = true;

                return true; // pattern locked in
            }
        }

        return false; // motif tag not in frame this call
    }

    // ================================================================
    // HELPER: Is this tag ID a motif tag (21, 22, or 23)?
    // ================================================================
    private boolean isMotifTag(int tagId) {
        return tagId >= MOTIF_TAG_MIN && tagId <= MOTIF_TAG_MAX;
    }

    // ================================================================
    // EXISTING METHODS (null-safe versions)
    // ================================================================

    /** Raw latest result — use getAprilTagData() for structured access. */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /** True if limelight currently sees any valid target on the active pipeline. */
    public boolean hasTarget() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    /**
     * Raw botpose from limelight — includes ALL tags (motif and nav).
     * Prefer getAprilTagData().poseValid for navigation use since that
     * filters motif tags.
     */
    public Pose3D getRobotPose() {
        LLResult result = getResult();
        return result != null ? result.getBotpose() : null;
    }

    /** Switch to any pipeline by index. */
    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /** Crosshair horizontal offset (degrees) from the active pipeline's primary target. */
    public double getTx() {
        LLResult result = getResult();
        return result != null ? result.getTx() : 0;
    }

    /** Crosshair vertical offset (degrees) from the active pipeline's primary target. */
    public double getTy() {
        LLResult result = getResult();
        return result != null ? result.getTy() : 0;
    }

    /** Target area as % of image from the active pipeline's primary target. */
    public double getTa() {
        LLResult result = getResult();
        return result != null ? result.getTa() : 0;
    }
}
package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LimelightConstants;

/**
 * Limelight 4 subsystem using NetworkTables (NT4).
 *
 * Key NT entries used:
 *   tv        — target valid (1 = tag visible, 0 = no target)
 *   tid       — primary tracked tag ID
 *   tx        — horizontal angle to target (degrees, + = right)
 *   ty        — vertical angle to target (degrees, + = up)
 *   ta        — target area (% of image)
 *   botpose_wpiblue — [x, y, z, roll, pitch, yaw, latency] robot pose in WPILib Blue field coords
 *   botpose_targetspace — pose of robot relative to primary tag
 */
public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    // Cached values updated every periodic() — avoids repeated NT lookups in commands
    private boolean targetValid      = false;
    private int     targetId         = -1;
    private double  tx               = 0.0;  // horizontal angle to tag (deg)
    private double  ty               = 0.0;  // vertical angle to tag (deg)
    private double  ta               = 0.0;  // target area
    private double  distanceToTag    = 0.0;  // calculated metres
    private Pose2d  robotPoseFromTag = new Pose2d(); // robot pose via MegaTag

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault()
            .getTable(LimelightConstants.LIMELIGHT_NAME);

        // Set pipeline to April Tag mode on init
        setPipeline(LimelightConstants.APRILTAG_PIPELINE);

        // Set LED mode to pipeline-controlled
        setLedMode(0);
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Read all core NT values once per loop
        targetValid = table.getEntry("tv").getDouble(0) == 1.0;
        targetId    = (int) table.getEntry("tid").getDouble(-1);
        tx          = table.getEntry("tx").getDouble(0.0);
        ty          = table.getEntry("ty").getDouble(0.0);
        ta          = table.getEntry("ta").getDouble(0.0);

        // Calculate distance from tag using vertical angle + known tag height
        if (targetValid) {
            distanceToTag = calculateDistance();
        }

        // Read MegaTag robot pose (field-relative, WPILib Blue alliance coords)
        double[] botpose = table.getEntry("botpose_wpiblue")
            .getDoubleArray(new double[7]);
        if (botpose.length >= 6 && targetValid) {
            robotPoseFromTag = new Pose2d(
                new Translation2d(botpose[0], botpose[1]),
                Rotation2d.fromDegrees(botpose[5]));
        }

        // SmartDashboard telemetry
        SmartDashboard.putBoolean("Limelight/TargetValid",   targetValid);
        SmartDashboard.putNumber("Limelight/TagID",          targetId);
        SmartDashboard.putNumber("Limelight/TX (deg)",       tx);
        SmartDashboard.putNumber("Limelight/TY (deg)",       ty);
        SmartDashboard.putNumber("Limelight/TA (%)",         ta);
        SmartDashboard.putNumber("Limelight/Distance (m)",   distanceToTag);
        SmartDashboard.putBoolean("Limelight/IsHubTag",      isHubTag());
        SmartDashboard.putBoolean("Limelight/InZone",        isInAlignmentZone());
    }

    // ─── Distance Calculation ─────────────────────────────────────────────────

    /**
     * Calculate distance to tag using vertical angle geometry.
     *
     * Formula: d = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
     *
     * All heights/angles defined in LimelightConstants.
     */
    private double calculateDistance() {
        double angleToTargetRad = Math.toRadians(
            LimelightConstants.CAMERA_MOUNT_ANGLE_DEG + ty);

        if (Math.abs(angleToTargetRad) < 0.001) return 0.0;

        return (LimelightConstants.TAG_HEIGHT_METERS - LimelightConstants.CAMERA_HEIGHT_METERS)
            / Math.tan(angleToTargetRad);
    }

    // ─── Hub Tag Detection ────────────────────────────────────────────────────

    /**
     * Returns true if the currently tracked tag is one of the hub tags.
     * Hub tag IDs: 2, 9, 10, 11 (set in LimelightConstants.HUB_TAG_IDS).
     */
    public boolean isHubTag() {
        if (!targetValid) return false;
        for (int id : LimelightConstants.HUB_TAG_IDS) {
            if (targetId == id) return true;
        }
        return false;
    }

    /**
     * Returns true when robot is within the arc alignment zone:
     *   - A hub tag is visible
     *   - Distance is within ARC_RADIUS_METERS
     */
    public boolean isInAlignmentZone() {
        return isHubTag() && distanceToTag > 0
            && distanceToTag <= LimelightConstants.ARC_RADIUS_METERS;
    }

    // ─── Limelight Config ─────────────────────────────────────────────────────

    public void setPipeline(int index) {
        table.getEntry("pipeline").setNumber(index);
    }

    /**
     * 0 = pipeline default, 1 = force off, 2 = force blink, 3 = force on
     */
    public void setLedMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    // ─── Getters ──────────────────────────────────────────────────────────────

    public boolean hasTarget()           { return targetValid; }
    public int     getTargetId()         { return targetId; }
    public double  getTx()               { return tx; }
    public double  getTy()               { return ty; }
    public double  getDistanceToTag()    { return distanceToTag; }
    public Pose2d  getRobotPoseFromTag() { return robotPoseFromTag; }
}
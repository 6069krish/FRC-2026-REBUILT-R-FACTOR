package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;


/**
 * AlignToHubCommand
 *
 * Drives the robot to exactly TARGET_DISTANCE_METERS (1 m) from the hub April Tag
 * and rotates until tx (horizontal offset) is 0° — robot is facing the tag squarely.
 *
 * Two independent PID loops:
 *   rotationPID  — corrects tx (yaw error, degrees → rad/s output)
 *   distancePID  — corrects range error (metres → m/s forward output)
 *
 * The command finishes when BOTH errors are within tolerance for
 * SETTLE_LOOPS consecutive loops (prevents premature exit on noisy readings).
 *
 * Conditions to run:
 *   - Limelight sees a hub tag (tv=1, tid in HUB_TAG_IDS)
 *   - Robot is inside the arc zone (distance ≤ ARC_RADIUS_METERS)
 *   - Operator holds the align button (PS5 controller, bound in RobotContainer)
 */
public class AlignToHubCommand extends Command {

    private final DriveSubsystem    drive;
    private final LimelightSubsystem limelight;

    // ── Rotation PID (tx error → rot speed) ──────────────────────────────────
    // Input: degrees (tx), Output: rad/s
    private final PIDController rotationPID = new PIDController(
        LimelightConstants.ALIGN_ROT_kP,
        LimelightConstants.ALIGN_ROT_kI,
        LimelightConstants.ALIGN_ROT_kD);

    // ── Distance PID (range error → forward speed) ────────────────────────────
    // Input: metres error, Output: m/s
    private final PIDController distancePID = new PIDController(
        LimelightConstants.ALIGN_DIST_kP,
        LimelightConstants.ALIGN_DIST_kI,
        LimelightConstants.ALIGN_DIST_kD);

    private int settleCounter = 0;

    public AlignToHubCommand(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive     = drive;
        this.limelight = limelight;
        addRequirements(drive); // takes over from DriveCommand while active

        rotationPID.setTolerance(LimelightConstants.ALIGN_ROT_TOLERANCE_DEG);
        distancePID.setTolerance(LimelightConstants.ALIGN_DIST_TOLERANCE_M);

        // tx setpoint = 0° (tag centred in camera)
        rotationPID.setSetpoint(0.0);
        // distance setpoint = 1 m from tag
        distancePID.setSetpoint(LimelightConstants.TARGET_DISTANCE_METERS);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        distancePID.reset();
        settleCounter = 0;
        SmartDashboard.putBoolean("AutoAlign/Active", true);
    }

    @Override
    public void execute() {
        // ── Guard: abort if tag lost ───────────────────────────────────────────
        if (!limelight.isHubTag()) {
            drive.stopModules();
            return;
        }

        double currentTx       = limelight.getTx();
        double currentDistance = limelight.getDistanceToTag();

        // ── Rotation correction ───────────────────────────────────────────────
        // tx > 0 → tag is to the right → rotate CW → negative rot output (WPILib CCW+)
        double rotOutput = -rotationPID.calculate(currentTx);
        rotOutput = MathUtil.clamp(rotOutput,
            -LimelightConstants.ALIGN_MAX_ROT_SPEED,
             LimelightConstants.ALIGN_MAX_ROT_SPEED);

        // ── Distance correction ───────────────────────────────────────────────
        // distancePID: error = target - current
        // positive error → too far → drive forward (positive x)
        // negative error → too close → drive backward (negative x)
        double fwdOutput = distancePID.calculate(currentDistance);
        fwdOutput = MathUtil.clamp(fwdOutput,
            -LimelightConstants.ALIGN_MAX_FWD_SPEED,
             LimelightConstants.ALIGN_MAX_FWD_SPEED);

        // Drive robot-relative (not field-relative) so alignment is tag-relative
        drive.drive(fwdOutput, 0.0, rotOutput, false);

        // ── Settle counter ────────────────────────────────────────────────────
        boolean atAngle    = Math.abs(currentTx) < LimelightConstants.ALIGN_ROT_TOLERANCE_DEG;
        boolean atDistance = Math.abs(currentDistance - LimelightConstants.TARGET_DISTANCE_METERS)
                             < LimelightConstants.ALIGN_DIST_TOLERANCE_M;

        if (atAngle && atDistance) {
            settleCounter++;
        } else {
            settleCounter = 0;
        }

        // Telemetry
        SmartDashboard.putNumber("AutoAlign/TX Error (deg)",   currentTx);
        SmartDashboard.putNumber("AutoAlign/Distance (m)",     currentDistance);
        SmartDashboard.putNumber("AutoAlign/DistError (m)",
            currentDistance - LimelightConstants.TARGET_DISTANCE_METERS);
        SmartDashboard.putNumber("AutoAlign/FwdOutput",        fwdOutput);
        SmartDashboard.putNumber("AutoAlign/RotOutput",        rotOutput);
        SmartDashboard.putNumber("AutoAlign/SettleCounter",    settleCounter);
        SmartDashboard.putBoolean("AutoAlign/Aligned",
            settleCounter >= LimelightConstants.ALIGN_SETTLE_LOOPS);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopModules();
        SmartDashboard.putBoolean("AutoAlign/Active",  false);
        SmartDashboard.putBoolean("AutoAlign/Aligned", false);
    }

    /**
     * Finish when both errors are within tolerance for SETTLE_LOOPS consecutive loops
     * AND the operator releases the button (whileTrue handles button release separately).
     */
    @Override
    public boolean isFinished() {
        return settleCounter >= LimelightConstants.ALIGN_SETTLE_LOOPS;
    }
}

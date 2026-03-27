package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.Constants.DriveConstants;

/**
 * Swerve drive subsystem: 4x Kraken X60, 4x CANcoder, Pigeon 2 IMU.
 *
 * Module order is FL, FR, BL, BR everywhere — matches KINEMATICS, setModuleStates(),
 * and getModulePositions().
 */
public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    // FIX: was hardcoded to CAN ID 0 — swervedrive.json specifies id: 19
    private final Pigeon2 pigeon;

    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();

    public DriveSubsystem() {
        frontLeft  = new SwerveModule(
            DriveConstants.FL_DRIVE_ID, DriveConstants.FL_STEER_ID,
            DriveConstants.FL_CANCODER_ID, DriveConstants.FL_ENCODER_OFFSET_ROT,
            DriveConstants.FL_DRIVE_INVERTED);

        frontRight = new SwerveModule(
            DriveConstants.FR_DRIVE_ID, DriveConstants.FR_STEER_ID,
            DriveConstants.FR_CANCODER_ID, DriveConstants.FR_ENCODER_OFFSET_ROT,
            DriveConstants.FR_DRIVE_INVERTED);

        backLeft   = new SwerveModule(
            DriveConstants.BL_DRIVE_ID, DriveConstants.BL_STEER_ID,
            DriveConstants.BL_CANCODER_ID, DriveConstants.BL_ENCODER_OFFSET_ROT,
            DriveConstants.BL_DRIVE_INVERTED);

        backRight  = new SwerveModule(
            DriveConstants.BR_DRIVE_ID, DriveConstants.BR_STEER_ID,
            DriveConstants.BR_CANCODER_ID, DriveConstants.BR_ENCODER_OFFSET_ROT,
            DriveConstants.BR_DRIVE_INVERTED);

        // FIX: use PIGEON_ID constant (19) instead of hardcoded 0
        pigeon = new Pigeon2(DriveConstants.PIGEON_ID, DriveConstants.CAN_BUS_NAME);
        pigeon.reset();

        odometry = new SwerveDriveOdometry(
            DriveConstants.KINEMATICS,
            getHeading(),
            getModulePositions());

        SmartDashboard.putData("Field", field);

        configurePathPlanner();
    }

    private void configurePathPlanner() {
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            ppConfig = null;
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            ppConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        odometry.update(getHeading(), getModulePositions());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Heading (deg)",  getHeading().getDegrees());
        SmartDashboard.putNumber("FL Angle (deg)", frontLeft.getAngle().getDegrees());
        SmartDashboard.putNumber("FR Angle (deg)", frontRight.getAngle().getDegrees());
        SmartDashboard.putNumber("BL Angle (deg)", backLeft.getAngle().getDegrees());
        SmartDashboard.putNumber("BR Angle (deg)", backRight.getAngle().getDegrees());
        SmartDashboard.putNumber("FL Speed (mps)", frontLeft.getDriveVelocityMps());
        SmartDashboard.putNumber("FR Speed (mps)", frontRight.getDriveVelocityMps());
        SmartDashboard.putNumber("BL Speed (mps)", backLeft.getDriveVelocityMps());
        SmartDashboard.putNumber("BR Speed (mps)", backRight.getDriveVelocityMps());
    }

    // ─── Drive API ────────────────────────────────────────────────────────────

    /**
     * @param xSpeed        Forward  [m/s]   +X = toward positive field X (forward)
     * @param ySpeed        Strafe   [m/s]   +Y = toward positive field Y (left)
     * @param rot           Rotation [rad/s] CCW positive
     * @param fieldRelative true = field-oriented driving
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rot, getHeading())
            : new ChassisSpeeds(ySpeed, xSpeed, rot);

        // discretize() compensates for translational drift caused by rotation over 20 ms
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_MPS);
        setModuleStates(states);
    }

    /** Drive using robot-relative ChassisSpeeds — called by PathPlanner. */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_MPS);
        setModuleStates(states);
    }

    /** Robot-relative ChassisSpeeds — used by PathPlanner for feedback. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("Expected 4 module states (FL, FR, BL, BR)");
        }
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // ─── Gyro ─────────────────────────────────────────────────────────────────

    /**
     * Robot heading, CCW positive (WPILib convention).
     *
     * Pigeon2 yaw increases CW when viewed from above, so we negate to get CCW+.
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Degrees));
    }

    public void zeroHeading() {
        pigeon.reset();
    }

    // ─── Odometry ─────────────────────────────────────────────────────────────

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
}

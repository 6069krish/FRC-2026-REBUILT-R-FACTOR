package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

/**
 * RotateToTagCommand
 *
 * Only rotates the robot to face the AprilTag (tx → 0).
 * No strafing, no driving forward — robot stays in place.
 *
 * Bind to a button in RobotContainer:
 *   driverController.L1().whileTrue(new RotateToTagCommand(limelightSubsystem, driveSubsystem));
 */
public class SnapToDiamond extends Command {

    private final DriveSubsystem  drivetrain;

    // 🔧 Tune kP first — increase until rotation is confident but not oscillating
    private final PIDController rotationPID = new PIDController(
        0.1,   // kP
        0.0,    // kI
        0.0   // kD
    );

    private static final double TX_TOLERANCE_DEG = 1.5;  // degrees — "facing it"
    private static final double MAX_ROT_RADPS    = 10;  // max rotation speed

    private double currentHeading = 0;
    private double setpoint = 45;

    public SnapToDiamond(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        rotationPID.setTolerance(TX_TOLERANCE_DEG);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
    }

    @Override
    public void execute() {
        double N = Math.round((currentHeading - 45.0) / 90.0);
        setpoint = 45.0 + 90.0 * N;
        currentHeading = drivetrain.getHeading().getDegrees();
        double rotSpeed = clamp(rotationPID.calculate(currentHeading, setpoint), -MAX_ROT_RADPS, MAX_ROT_RADPS);
        // Only rotation — x and y speeds are 0
        drivetrain.drive(0, 0, rotSpeed, false);

        SmartDashboard.putNumber("RotateToTag Current Heading",drivetrain.getHeading().getDegrees());
        SmartDashboard.putNumber("RotateToTag rotSpeed", rotSpeed);
        SmartDashboard.putNumber("Setpoint", setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint-currentHeading) < TX_TOLERANCE_DEG;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private final DriveSubsystem drive;
    private final CommandPS5Controller controller;

    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter   = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2.0 * Math.PI);

    private boolean fieldRelative    = true;
    private boolean lastOptionsState = false;

    public DriveCommand(DriveSubsystem drive, CommandPS5Controller controller) {
        this.drive      = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotLimiter.reset(0);
        lastOptionsState = false;
    }

    @Override
    public void execute() {
        boolean optionsNow = controller.options().getAsBoolean();
        if (optionsNow && !lastOptionsState) fieldRelative = !fieldRelative;
        lastOptionsState = optionsNow;

        double xSpeed   = applyDeadband(controller.getLeftY());
        double ySpeed   = applyDeadband(controller.getLeftX());
        double rotSpeed = applyDeadband(controller.getRightX());

        xSpeed   = xLimiter.calculate(xSpeed   * DriveConstants.MAX_SPEED_MPS);
        ySpeed   = yLimiter.calculate(ySpeed   * DriveConstants.MAX_SPEED_MPS);
        rotSpeed = rotLimiter.calculate(rotSpeed * DriveConstants.MAX_ANGULAR_SPEED_RPS);

        drive.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applyDeadband(double value) {
        double db = OIConstants.DEADBAND;
        if (Math.abs(value) < db) return 0.0;
        return Math.signum(value) * (Math.abs(value) - db) / (1.0 - db);
    }
}
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.ArcShootCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.LUTAutoShootCommand;
import frc.robot.Commands.SnapToDiamond;
import frc.robot.Commands.ShootEightBallsCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

    // DriveSubsystem MUST be instantiated first — its constructor calls AutoBuilder.configure()
    private final DriveSubsystem     driveSubsystem     = new DriveSubsystem();
    private final IntakeSubsystem    intakeSubsystem    = new IntakeSubsystem();
    private final FeederSubsystem    feederSubsystem    = new FeederSubsystem();
    private final ShooterSubsystem   shooterSubsystem   = new ShooterSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    private final CommandPS5Controller driverController =
        new CommandPS5Controller(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS5Controller operatorController =
        new CommandPS5Controller(OIConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ShootEightBalls",
            new ShootEightBallsCommand(driveSubsystem, limelightSubsystem, shooterSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Published once at startup — Shuffleboard edits persist across presses
        SmartDashboard.putNumber("AutoShoot/ManualRPS", 0.0);

        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverController));

        configureBindings();
    }

    private void configureBindings() {

        // ── Driver ────────────────────────────────────────────────────────────
        driverController.options().onTrue(
            new InstantCommand(driveSubsystem::zeroHeading, driveSubsystem));

        driverController.L1().onTrue(
            new SnapToDiamond(driveSubsystem));

        // ── Operator — Square: intake outtake ─────────────────────────────────
        operatorController.L1().whileTrue(
            new StartEndCommand(
                intakeSubsystem::runRollerOuttake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));

        // ── Operator — Cross: intake in ───────────────────────────────────────
        operatorController.L2().whileTrue(
            new StartEndCommand(
                intakeSubsystem::runRollerIntake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));

        // ── Operator — R1: feeder (fire) ──────────────────────────────────────
        operatorController.R1().whileTrue(
            new StartEndCommand(
                feederSubsystem::runFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));

        // ── Operator — L1: feeder reverse (unjam) ────────────────────────────
        operatorController.triangle().whileTrue(
            new StartEndCommand(
                feederSubsystem::reverseFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));

        // ── Operator — R1: reverse shooter (unjam) ───────────────────────────
        operatorController.cross().onTrue(
            new RunCommand(shooterSubsystem::reverse, shooterSubsystem)
        ).onFalse(
            new InstantCommand(shooterSubsystem::stop, shooterSubsystem)
        );

        // operatorController.circle().onTrue(
        //     new RunCommand(shooterSubsystem::column, shooterSubsystem)
        // ).onFalse(
        //     new InstantCommand(shooterSubsystem::stop1, shooterSubsystem)
        // );

        // operatorController.R3().whileTrue(
        // new ArcShootCommand(limelightSubsystem, driveSubsystem,
        //                   shooterSubsystem, operatorController));
        operatorController.R2().whileTrue(
        new LUTAutoShootCommand(limelightSubsystem,shooterSubsystem));

 
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

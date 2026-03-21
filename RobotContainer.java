package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Commands.AlignToHubCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

/**
 * RobotContainer wires all subsystems, commands, and controller bindings.
 *
 * Controller 0 (Driver) - Swerve:
 *   Left  stick       -> translate (field-relative)
 *   Right stick X     -> rotate
 *   Options           -> toggle field-relative (edge-detected in DriveCommand)
 *   Touchpad          -> zero gyro heading
 *   PS (hold)         -> auto-align to hub April Tag (when in arc zone)
 *
 * Controller 1 (Operator) - Mechanisms:
 *   R2  (hold)        -> deploy arm + intake rollers IN
 *   L2  (hold)        -> deploy arm + rollers OUT (eject)
 *   Circle            -> retract / stow arm
 *   R1  (hold)        -> feeder forward
 *   L1  (hold)        -> feeder reverse (unjam)
 *   Triangle (hold)   -> spin up shooter
 *   Square   (hold)   -> reverse shooter (unjam)
 */
public class RobotContainer {

    // DriveSubsystem MUST be instantiated first — its constructor calls AutoBuilder.configure()
    private final DriveSubsystem   driveSubsystem   = new DriveSubsystem();
    private final IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem();
    private final FeederSubsystem  feederSubsystem  = new FeederSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    private final CommandPS5Controller driverController   =
        new CommandPS5Controller(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS5Controller operatorController =
        new CommandPS5Controller(OIConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
       
        autoChooser = AutoBuilder.buildAutoChooser("GO_FORWARD.path");
        SmartDashboard.putData("Auto Chooser", autoChooser);    

        driveSubsystem.setDefaultCommand(
            new DriveCommand(driveSubsystem, driverController));

        configureBindings();
    }

    private void configureBindings() {

        // ── Driver — Swerve ───────────────────────────────────────────────────
        // Touchpad -> zero gyro (instant, non-blocking)
        // driverController.povUp().onTrue(
        //     new InstantCommand(driveSubsystem::zeroHeading, driveSubsystem));

        // ── Driver — Auto Align to Hub ────────────────────────────────────────
        // PS button held + robot inside arc zone → run AlignToHubCommand
        // When button is released OR alignment completes, command ends and
        // DriveCommand (default) resumes automatically.
        //
        // Trigger = PS button AND limelight sees a hub tag AND robot is in zone
        edu.wpi.first.wpilibj2.command.button.Trigger alignTrigger =
            driverController.PS()
                .and(() -> limelightSubsystem.isInAlignmentZone());

        alignTrigger.whileTrue(
            new AlignToHubCommand(driveSubsystem, limelightSubsystem));

        // ── Operator — Intake ─────────────────────────────────────────────────
        // R2 held -> deploy arm + rollers IN
        operatorController.R2().whileTrue(
            new StartEndCommand(
                () -> { intakeSubsystem.deploy(); intakeSubsystem.runRollerIntake(); },
                ()  -> intakeSubsystem.stopRoller(),
                intakeSubsystem));

        // L2 held -> deploy arm + rollers OUT (eject)
        operatorController.L2().whileTrue(
            new StartEndCommand(
                () -> { intakeSubsystem.deploy(); intakeSubsystem.runRollerOuttake(); },
                ()  -> intakeSubsystem.stopRoller(),
                intakeSubsystem));

        // Circle -> retract / stow arm
        operatorController.circle().onTrue(
            new InstantCommand(intakeSubsystem::retract, intakeSubsystem));

        // ── Operator — Feeder ─────────────────────────────────────────────────
        operatorController.R1().whileTrue(
            new StartEndCommand(
                feederSubsystem::runFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));

        operatorController.L1().whileTrue(
            new StartEndCommand(
                feederSubsystem::reverseFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));

        // ── Operator — Shooter ────────────────────────────────────────────────
        operatorController.triangle().whileTrue(
            new StartEndCommand(
                shooterSubsystem::spinUp,
                shooterSubsystem::stop,
                shooterSubsystem));

        operatorController.square().whileTrue(
            new StartEndCommand(
                shooterSubsystem::reverse,
                shooterSubsystem::stop,
                shooterSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
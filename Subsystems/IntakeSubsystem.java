package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem:
 *   CAN 25 — Roller motor  : spins to intake or outtake the game piece (duty cycle)
 *   CAN 27 — Pivot motor   : rotates the arm in/out using PID position control
 */
public class IntakeSubsystem extends SubsystemBase {

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX rollerMotor;
    private final TalonFX pivotMotor;

    // ─── Control requests ─────────────────────────────────────────────────────
    private final DutyCycleOut   rollerReq = new DutyCycleOut(0);
    private final PositionVoltage pivotReq = new PositionVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID, DriveConstants.CAN_BUS_NAME);
        pivotMotor  = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID,  DriveConstants.CAN_BUS_NAME);

        configureRoller();
        configurePivot();

        // Seed pivot position to 0 on startup (assumes arm starts retracted)
        pivotMotor.setPosition(0);
    }

    // ─── Configuration ────────────────────────────────────────────────────────

    private void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.ROLLER_SUPPLY_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers coast when idle
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        rollerMotor.getConfigurator().apply(cfg);
    }

    private void configurePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.PIVOT_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = IntakeConstants.PIVOT_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // hold position when idle
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        // Gear ratio: motor rotations → mechanism rotations
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;

        // Position PID with gravity feedforward (arm = ArmCosine compensates for angle)
        cfg.Slot0.kP          = IntakeConstants.PIVOT_kP;
        cfg.Slot0.kI          = IntakeConstants.PIVOT_kI;
        cfg.Slot0.kD          = IntakeConstants.PIVOT_kD;
        cfg.Slot0.kG          = IntakeConstants.PIVOT_kG;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Soft limits — prevent arm from going past physical stops
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_DEPLOYED_ROT + 0.5;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_RETRACTED_ROT - 0.5;

        pivotMotor.getConfigurator().apply(cfg);
    }

    // ─── Roller API ───────────────────────────────────────────────────────────

    /** Run rollers inward to collect a game piece. */
    public void runRollerIntake() {
        rollerMotor.setControl(rollerReq.withOutput(IntakeConstants.ROLLER_INTAKE_SPEED));
    }

    /** Run rollers outward to eject a game piece. */
    public void runRollerOuttake() {
        rollerMotor.setControl(rollerReq.withOutput(IntakeConstants.ROLLER_OUTTAKE_SPEED));
    }

    /** Stop rollers. */
    public void stopRoller() {
        rollerMotor.setControl(rollerReq.withOutput(0));
    }

    // ─── Pivot API ────────────────────────────────────────────────────────────

    /** Deploy arm to the ground/intake position. */
    public void deploy() {
        pivotMotor.setControl(pivotReq.withPosition(IntakeConstants.PIVOT_DEPLOYED_ROT));
    }

    /** Retract arm to the stowed/inside position. */
    public void retract() {
        pivotMotor.setControl(pivotReq.withPosition(IntakeConstants.PIVOT_RETRACTED_ROT));
    }

    /** Returns true when the pivot is within tolerance of the deployed position. */
    public boolean isDeployed() {
        double pos = pivotMotor.getPosition().getValue().in(Units.Rotations);
        return Math.abs(pos - IntakeConstants.PIVOT_DEPLOYED_ROT) < 0.5;
    }

    /** Returns true when the pivot is within tolerance of the retracted position. */
    public boolean isRetracted() {
        double pos = pivotMotor.getPosition().getValue().in(Units.Rotations);
        return Math.abs(pos - IntakeConstants.PIVOT_RETRACTED_ROT) < 0.5;
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position (rot)",
            pivotMotor.getPosition().getValue().in(Units.Rotations));
        SmartDashboard.putNumber("Intake/Roller Current (A)",
            rollerMotor.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putBoolean("Intake/Deployed",  isDeployed());
        SmartDashboard.putBoolean("Intake/Retracted", isRetracted());
    }
}
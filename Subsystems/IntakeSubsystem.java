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

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;

    private final DutyCycleOut   rollerReq = new DutyCycleOut(0);
    // withSlot(0) = use the PID gains in Slot0
    // withEnableFOC(true) = better torque control on Kraken
    private final PositionVoltage pivotReq = new PositionVoltage(0)
        .withSlot(0)
        .withEnableFOC(true);

    public IntakeSubsystem() {
        pivotMotor  = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID,  DriveConstants.CAN_BUS_NAME);
        rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID, DriveConstants.CAN_BUS_NAME);

        configurePivot();
        configureRoller();

        // Zero the pivot encoder on boot — assumes pivot starts retracted
        pivotMotor.setPosition(0.0);
    }

    // ─── Configuration ────────────────────────────────────────────────────────

    private void configurePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.PIVOT_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = IntakeConstants.PIVOT_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        // Gear ratio: rotor turns per one output shaft rotation
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;

        // Slot0: positional PID + gravity feedforward
        cfg.Slot0.kP = IntakeConstants.PIVOT_kP;
        cfg.Slot0.kI = IntakeConstants.PIVOT_kI;
        cfg.Slot0.kD = IntakeConstants.PIVOT_kD;

        // kG: gravity compensation — ArmCosine means output = kG * cos(angle)
        // This counteracts gravity as the arm rotates, keeping it from falling
        cfg.Slot0.kG = IntakeConstants.PIVOT_kG;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Soft limits: prevent pivot from going past physical stops
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_DEPLOYED_ROT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_RETRACTED_ROT;

        pivotMotor.getConfigurator().apply(cfg);
    }

    private void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.ROLLER_SUPPLY_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        rollerMotor.getConfigurator().apply(cfg);
    }

    // ─── Pivot API ────────────────────────────────────────────────────────────

    /** Deploy intake to ground position. */
    public void deployPivot() {
        pivotMotor.setControl(pivotReq.withPosition(IntakeConstants.PIVOT_DEPLOYED_ROT));
    }

    /** Retract intake to stowed position. */
    public void retractPivot() {
        pivotMotor.setControl(pivotReq.withPosition(IntakeConstants.PIVOT_RETRACTED_ROT));
    }

    /** Go to an arbitrary angle (rotations, output shaft). */
    public void setPivotPosition(double rotations) {
        pivotMotor.setControl(pivotReq.withPosition(rotations));
    }

    /** True when pivot is within a small tolerance of its target. */
    public boolean isPivotAtTarget(double targetRot) {
        double current = pivotMotor.getPosition().getValue().in(Units.Rotations);
        return Math.abs(current - targetRot) < 0.2; // ±0.2 rot tolerance — tune as needed
    }

    // ─── Roller API ───────────────────────────────────────────────────────────

    public void runRollerIntake() {
        rollerMotor.setControl(rollerReq.withOutput(IntakeConstants.ROLLER_INTAKE_SPEED));
    }

    public void runRollerOuttake() {
        rollerMotor.setControl(rollerReq.withOutput(IntakeConstants.ROLLER_OUTTAKE_SPEED));
    }

    public void stopRoller() {
        rollerMotor.setControl(rollerReq.withOutput(0));
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position (rot)",
            pivotMotor.getPosition().getValue().in(Units.Rotations));
        SmartDashboard.putNumber("Intake/Pivot Current (A)",
            pivotMotor.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Intake/Roller Current (A)",
            rollerMotor.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putBoolean("Intake/PivotDeployed",
            isPivotAtTarget(IntakeConstants.PIVOT_DEPLOYED_ROT));
        SmartDashboard.putBoolean("Intake/PivotRetracted",
            isPivotAtTarget(IntakeConstants.PIVOT_RETRACTED_ROT));
    }
}

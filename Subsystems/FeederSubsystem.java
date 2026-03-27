package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Feeder subsystem:
 *   CAN 26 — Feeds game piece from intake toward the shooter column.
 *   Shooter4 — Runs alongside feeder to assist game piece delivery.
 */
public class FeederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor;
    private final TalonFX shooter4;

    private final DutyCycleOut    feederReq   = new DutyCycleOut(0);
    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final DutyCycleOut    stopReq     = new DutyCycleOut(0);

    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_ID, DriveConstants.CAN_BUS_NAME);
        shooter4    = new TalonFX(ShooterConstants.SHOOTER_4_ID, DriveConstants.CAN_BUS_NAME);

        configureFeeder();
        configureShooter4();
    }

    private void configureFeeder() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = FeederConstants.FEEDER_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = FeederConstants.FEEDER_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        feederMotor.getConfigurator().apply(cfg);
    }

    private void configureShooter4() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.SHOOTER_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        cfg.Slot0.kP = ShooterConstants.SHOOTER_kP;
        cfg.Slot0.kI = ShooterConstants.SHOOTER_kI;
        cfg.Slot0.kD = ShooterConstants.SHOOTER_kD;
        cfg.Slot0.kV = ShooterConstants.SHOOTER_kV;
        cfg.Slot0.kS = ShooterConstants.SHOOTER_kS;

        shooter4.getConfigurator().apply(cfg);
    }

    // ─── API ──────────────────────────────────────────────────────────────────

    /** Run feeder forward and spin shooter4 to push game piece toward shooter. */
    public void runFeeder() {
        feederMotor.setControl(feederReq.withOutput(FeederConstants.FEEDER_SPEED));
        // shooter4.setControl(velocityReq.withVelocity(-ShooterConstants.SHOOTER_4_INTAKE_RPS));
    }

    /** Run feeder in reverse and reverse shooter4 to unjam. */
    public void reverseFeeder() {
        feederMotor.setControl(feederReq.withOutput(FeederConstants.FEEDER_REVERSE_SPEED));
        // shooter4.setControl(velocityReq.withVelocity(ShooterConstants.REVERSE_VELOCITY_RPS));
    }

    /** Stop feeder and shooter4. */
    public void stopFeeder() {
        feederMotor.setControl(feederReq.withOutput(0));
        shooter4.setControl(stopReq);
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/Current (A)",
            feederMotor.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Feeder/Velocity (rps)",
            feederMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Feeder/Shooter4 Velocity (rps)",
            shooter4.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Feeder/Shooter4 Current (A)",
            shooter4.getSupplyCurrent().getValue().in(Units.Amps));
    }
}

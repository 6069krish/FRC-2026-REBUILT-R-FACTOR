package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;

/**
 * Feeder subsystem:
 *   CAN 26 — Feeds game piece from intake toward the shooter column.
 */
public class FeederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor;
    private final DutyCycleOut feederReq = new DutyCycleOut(0);

    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_ID, DriveConstants.CAN_BUS_NAME);
        configureFeeder();
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

    // ─── API ──────────────────────────────────────────────────────────────────

    /** Run feeder forward to push game piece toward shooter. */
    public void runFeeder() {
        feederMotor.setControl(feederReq.withOutput(FeederConstants.FEEDER_SPEED));
    }

    /** Run feeder in reverse to unjam. */
    public void reverseFeeder() {
        feederMotor.setControl(feederReq.withOutput(FeederConstants.FEEDER_REVERSE_SPEED));
    }

    /** Stop feeder. */
    public void stopFeeder() {
        feederMotor.setControl(feederReq.withOutput(0));
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/Current (A)",
            feederMotor.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Feeder/Velocity (rps)",
            feederMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
    }
}

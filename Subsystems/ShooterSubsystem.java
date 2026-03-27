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
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final TalonFX shooter3;
    private final TalonFX shooter4;

    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final DutyCycleOut    stopReq     = new DutyCycleOut(0);

    public ShooterSubsystem() {
        shooter1 = new TalonFX(ShooterConstants.SHOOTER_1_ID, DriveConstants.CAN_BUS_NAME);
        shooter2 = new TalonFX(ShooterConstants.SHOOTER_2_ID, DriveConstants.CAN_BUS_NAME);
        shooter3 = new TalonFX(ShooterConstants.SHOOTER_3_ID, DriveConstants.CAN_BUS_NAME);
        shooter4 = new TalonFX(ShooterConstants.SHOOTER_4_ID, DriveConstants.CAN_BUS_NAME);
        

        configureMotor(shooter1, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooter2, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooter3, InvertedValue.CounterClockwise_Positive);

        SmartDashboard.putNumber ("ShotVelocity", 0);
    }

    private void configureMotor(TalonFX motor, InvertedValue invert) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.SHOOTER_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted    = invert;

        cfg.Slot0.kP = ShooterConstants.SHOOTER_kP;
        cfg.Slot0.kI = ShooterConstants.SHOOTER_kI;
        cfg.Slot0.kD = ShooterConstants.SHOOTER_kD;
        cfg.Slot0.kV = ShooterConstants.SHOOTER_kV;
        cfg.Slot0.kS = ShooterConstants.SHOOTER_kS;

        motor.getConfigurator().apply(cfg);
    }

    // ─── API ──────────────────────────────────────────────────────────────────

    /** Spin up to fixed shoot velocity from Constants (manual shoot). */
    public void spinUp() {
        setVelocity(ShooterConstants.SHOOT_VELOCITY_RPS);
    }

    /**
     * Spin up to a DYNAMIC velocity — used by AutoShootCommand.
     * All 3 motors run at the same RPS.
     */
    public void setVelocity(double rps) {
        shooter1.setControl(velocityReq.withVelocity(rps));
        shooter2.setControl(velocityReq.withVelocity(-rps));
        shooter3.setControl(velocityReq.withVelocity(rps));
        // shooter4.setControl(velocityReq.withVelocity(rps));
    }
    public void column(){
        shooter4.setControl(velocityReq.withVelocity(-100));
    }

    /** Reverse all motors (unjam). */
    public void reverse() {
        double target = ShooterConstants.REVERSE_VELOCITY_RPS;
        shooter1.setControl(velocityReq.withVelocity(target));
        shooter2.setControl(velocityReq.withVelocity(-target));
        shooter3.setControl(velocityReq.withVelocity(target));
        shooter4.setControl(velocityReq.withVelocity(-target));
    }

    /** Stop all motors. */
    public void stop() {
        shooter1.setControl(stopReq);
        shooter2.setControl(stopReq);
        shooter3.setControl(stopReq);
        shooter4.setControl(stopReq);
    }
    public void stop1(){
        shooter4.setControl(stopReq);
    }

    /**
     * Check if all motors are within tolerance of a FIXED target (Constants).
     * Used by manual spinUp() check.
     */
    public boolean isAtSpeed() {
        return isAtSpeed(ShooterConstants.SHOOT_VELOCITY_RPS);
    }

    /**
     * Check if all motors are within tolerance of a DYNAMIC target.
     * Used by AutoShootCommand.
     */
    public boolean isAtSpeed(double targetRPS) {
        double tol = ShooterConstants.VELOCITY_TOLERANCE_RPS;
        double v1  = Math.abs(shooter1.getVelocity().getValue().in(Units.RotationsPerSecond));
        double v2  = Math.abs(shooter2.getVelocity().getValue().in(Units.RotationsPerSecond));
        double v3  = Math.abs(shooter3.getVelocity().getValue().in(Units.RotationsPerSecond));
        return (Math.abs(v1 - targetRPS) < tol);
        // return Math.abs(v1 - targetRPS) < tol
        //     && Math.abs(v2 - targetRPS) < tol
        //     && Math.abs(v3 - targetRPS) < tol;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/AtSpeed",      isAtSpeed());
        SmartDashboard.putNumber ("Shooter/Velocity1",    shooter1.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber ("Shooter/Velocity2",    shooter2.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber ("Shooter/Velocity3",    shooter3.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber ("Shooter/Current1 (A)", shooter1.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber ("Shooter/Current2 (A)", shooter2.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber ("Shooter/Current3 (A)", shooter3.getSupplyCurrent().getValue().in(Units.Amps));
    }
}

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

/**
 * Shooter / Column subsystem:
 *   CAN 21 — Shooter motor 1
 *   CAN 22 — Shooter motor 2
 *   CAN 23 — Shooter motor 3
 *
 * All three motors run at the same velocity target (velocity closed-loop).
 * Motor 1 is the leader; motors 2 and 3 mirror its setpoint independently
 * (no follower API used — keeps each motor's current/telemetry visible).
 */
public class ShooterSubsystem extends SubsystemBase {

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final TalonFX shooter3;
    private final TalonFX shooter4; // Kraken X44 — column intake (CAN 24)

    // ─── Control requests ─────────────────────────────────────────────────────
    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final DutyCycleOut    stopReq     = new DutyCycleOut(0);

    private boolean isSpunUp = false;

    public ShooterSubsystem() {
        shooter1 = new TalonFX(ShooterConstants.SHOOTER_1_ID, DriveConstants.CAN_BUS_NAME);
        shooter2 = new TalonFX(ShooterConstants.SHOOTER_2_ID, DriveConstants.CAN_BUS_NAME);
        shooter3 = new TalonFX(ShooterConstants.SHOOTER_3_ID, DriveConstants.CAN_BUS_NAME);
        shooter4 = new TalonFX(ShooterConstants.SHOOTER_4_ID, DriveConstants.CAN_BUS_NAME);

        configureMotor(shooter1, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooter2, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooter3, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooter4, InvertedValue.CounterClockwise_Positive);
    }

    private void configureMotor(TalonFX motor, InvertedValue invert) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.SHOOTER_STATOR_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; // flywheels should coast down
        cfg.MotorOutput.Inverted    = invert;

        // Velocity PID + feedforward (Slot 0)
        cfg.Slot0.kP = ShooterConstants.SHOOTER_kP;
        cfg.Slot0.kI = ShooterConstants.SHOOTER_kI;
        cfg.Slot0.kD = ShooterConstants.SHOOTER_kD;
        cfg.Slot0.kV = ShooterConstants.SHOOTER_kV;
        cfg.Slot0.kS = ShooterConstants.SHOOTER_kS;

        motor.getConfigurator().apply(cfg);
    }

    // ─── API ──────────────────────────────────────────────────────────────────

    /** Spin up flywheels (1-3) to shoot velocity; shooter4 runs as column intake feeding into flywheel. */
    public void spinUp() {
        double shootTarget  = ShooterConstants.SHOOT_VELOCITY_RPS;
        double intakeTarget = ShooterConstants.SHOOTER_4_INTAKE_RPS;
        shooter1.setControl(velocityReq.withVelocity(shootTarget));
        shooter2.setControl(velocityReq.withVelocity(shootTarget));
        shooter3.setControl(velocityReq.withVelocity(shootTarget));
        shooter4.setControl(velocityReq.withVelocity(intakeTarget)); // feeds piece into flywheel
    }

    /** Run all 4 motors in reverse (unjam / feed back out). */
    public void reverse() {
        double target = ShooterConstants.REVERSE_VELOCITY_RPS;
        shooter1.setControl(velocityReq.withVelocity(target));
        shooter2.setControl(velocityReq.withVelocity(target));
        shooter3.setControl(velocityReq.withVelocity(target));
        shooter4.setControl(velocityReq.withVelocity(target));
    }

    /** Stop all shooter motors. */
    public void stop() {
        shooter1.setControl(stopReq);
        shooter2.setControl(stopReq);
        shooter3.setControl(stopReq);
        shooter4.setControl(stopReq);
    }

    /**
     * Returns true when all 3 motors are within velocity tolerance.
     * Use this to gate the feeder — only feed when shooter is up to speed.
     */
    public boolean isAtSpeed() {
        double target = ShooterConstants.SHOOT_VELOCITY_RPS;
        double tol    = ShooterConstants.VELOCITY_TOLERANCE_RPS;
        double v1 = shooter1.getVelocity().getValue().in(Units.RotationsPerSecond);
        double v2 = shooter2.getVelocity().getValue().in(Units.RotationsPerSecond);
        double v3 = shooter3.getVelocity().getValue().in(Units.RotationsPerSecond);
        double v4 = shooter4.getVelocity().getValue().in(Units.RotationsPerSecond);
        return Math.abs(v1 - target) < tol
            && Math.abs(v2 - target) < tol
            && Math.abs(v3 - target) < tol
            && Math.abs(v4 - target) < tol;
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        isSpunUp = isAtSpeed();
        SmartDashboard.putBoolean("Shooter/AtSpeed",      isSpunUp);
        SmartDashboard.putNumber("Shooter/Velocity1",     shooter1.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Velocity2",     shooter2.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Velocity3",     shooter3.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Velocity4",     shooter4.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Current1 (A)",  shooter1.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Shooter/Current2 (A)",  shooter2.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Shooter/Current3 (A)",  shooter3.getSupplyCurrent().getValue().in(Units.Amps));
        SmartDashboard.putNumber("Shooter/Current4 (A)",  shooter4.getSupplyCurrent().getValue().in(Units.Amps));
    }
}
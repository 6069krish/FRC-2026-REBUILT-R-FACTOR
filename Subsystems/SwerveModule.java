package frc.robot.Subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants.DriveConstants;

/**
 * One swerve module: Kraken X60 drive + Kraken X60 steer + CTRE CANcoder absolute.
 *
 * Design notes:
 *   - CANcoder MagnetOffset is set in code so all offset management is in Constants.
 *   - Steer TalonFX uses RemoteCANcoder feedback (no Phoenix Pro licence needed).
 *   - ContinuousWrap ensures the PID always takes the shorter than or equal to 180 path.
 *   - seedSteerPosition() syncs the TalonFX rotor register to CANcoder on every boot.
 *   - Cosine scaling reduces drive speed during large steer moves to prevent wheel scrub.
 */
public class SwerveModule {

    private final TalonFX  driveMotor;
    private final TalonFX  steerMotor;
    private final CANcoder absEncoder;

    private final VelocityVoltage driveVelocityReq =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
    private final PositionVoltage steerPositionReq =
        new PositionVoltage(0).withSlot(0).withEnableFOC(false);
    private final NeutralOut neutralOut = new NeutralOut();

    public SwerveModule(
        int driveId,
        int steerId,
        int cancoderId,
        double encoderOffsetRot,
        boolean driveInverted)
    {
        driveMotor = new TalonFX(driveId,     DriveConstants.CAN_BUS_NAME);
        steerMotor = new TalonFX(steerId,     DriveConstants.CAN_BUS_NAME);
        absEncoder = new CANcoder(cancoderId, DriveConstants.CAN_BUS_NAME);

        configureCANcoder(encoderOffsetRot);
        configureSteerMotor();
        configureDriveMotor(driveInverted);
        seedSteerPosition();
    }

    // -------------------------------------------------------------------------

    private void configureCANcoder(double encoderOffsetRot) {
        CANcoderConfiguration cfg = new CANcoderConfiguration();

        // MagnetOffset = negative of the raw forward reading so 0 rot = wheels forward.
        cfg.MagnetSensor.MagnetOffset = -encoderOffsetRot;

        // +/-0.5 rot discontinuity (standard swerve range)
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        // CCW positive matches WPILib field coordinate convention
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absEncoder.getConfigurator().apply(cfg);
    }

    private void configureSteerMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 20;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = 40;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // All four steer motors are inverted per the JSON files
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // RemoteCANcoder: sensor fusion without Phoenix Pro licence
        cfg.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = absEncoder.getDeviceID();
        cfg.Feedback.RotorToSensorRatio     = DriveConstants.STEER_GEAR_RATIO;

        cfg.Slot0.kP = DriveConstants.STEER_kP;
        cfg.Slot0.kI = DriveConstants.STEER_kI;
        cfg.Slot0.kD = DriveConstants.STEER_kD;

        // ContinuousWrap: always rotate the short way
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        steerMotor.getConfigurator().apply(cfg);
    }

    private void configureDriveMotor(boolean driveInverted) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLowerLimit  = 80;
        cfg.CurrentLimits.SupplyCurrentLowerTime   = 0.1;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = 80;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted    = driveInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        cfg.Slot0.kP = DriveConstants.DRIVE_kP;
        cfg.Slot0.kI = DriveConstants.DRIVE_kI;
        cfg.Slot0.kD = DriveConstants.DRIVE_kD;
        cfg.Slot0.kV = DriveConstants.DRIVE_kV;
        cfg.Slot0.kS = DriveConstants.DRIVE_kS;

        // SensorToMechanismRatio: rotor turns per one wheel revolution
        cfg.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;

        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod     = 0.25;
        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

        driveMotor.getConfigurator().apply(cfg);
        driveMotor.setPosition(0.0);
    }

    /**
     * Angle-drift fix: wait for a fresh CANcoder frame then seed the steer TalonFX
     * position register to match, so every boot starts aligned regardless of rotor
     * position at power-on.
     *
     * FIX vs original: setPosition() takes CANcoder (output-shaft) rotations, not rotor
     * rotations. The TalonFX firmware scales internally via RotorToSensorRatio.
     * The original code incorrectly multiplied by STEER_GEAR_RATIO here, causing the
     * TalonFX to initialize with a position 21x too large.
     */
    private void seedSteerPosition() {
        var absSignal = absEncoder.getAbsolutePosition();
        BaseStatusSignal.waitForAll(0.1, absSignal);
        double absRotations = absSignal.getValue().in(Rotations);
        // Pass CANcoder (sensor) rotations — firmware multiplies by RotorToSensorRatio internally
        steerMotor.setPosition(absRotations);
    }

    // -------------------------------------------------------------------------

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getAngle());
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getAngle()).getCos();

        double driveRotPerSec = desiredState.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE;
        driveMotor.setControl(driveVelocityReq.withVelocity(driveRotPerSec));
        steerMotor.setControl(steerPositionReq.withPosition(desiredState.angle.getRotations()));
    }

    /** Current wheel angle from CANcoder (MagnetOffset already applied). */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
            absEncoder.getAbsolutePosition().getValue().in(Rotations));
    }

    public double getDrivePositionMeters() {
        return driveMotor.getPosition().getValue().in(Rotations) * DriveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getDriveVelocityMps() {
        return driveMotor.getVelocity().getValue().in(RotationsPerSecond) * DriveConstants.WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMps(), getAngle());
    }

    public void stop() {
        driveMotor.setControl(neutralOut);
        steerMotor.setControl(neutralOut);
    }
}
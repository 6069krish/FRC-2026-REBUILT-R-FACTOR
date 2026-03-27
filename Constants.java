package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveConstants {
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21.75);
        public static final double WHEEL_BASE_METERS  = Units.inchesToMeters(21.75);

        public static final int FL_DRIVE_ID = 5;
        public static final int FR_DRIVE_ID = 7;
        public static final int BL_DRIVE_ID = 3;
        public static final int BR_DRIVE_ID = 1;

        public static final int FL_STEER_ID = 6;
        public static final int FR_STEER_ID = 8;
        public static final int BL_STEER_ID = 4;
        public static final int BR_STEER_ID = 2;

        public static final int FL_CANCODER_ID = 17;
        public static final int FR_CANCODER_ID = 18;
        public static final int BL_CANCODER_ID = 16;
        public static final int BR_CANCODER_ID = 15;

        public static final int PIGEON_ID = 19;

        public static final boolean FL_DRIVE_INVERTED = true;
        public static final boolean FR_DRIVE_INVERTED = true;
        public static final boolean BL_DRIVE_INVERTED = true;
        public static final boolean BR_DRIVE_INVERTED = false;

        public static final double FL_ENCODER_OFFSET_ROT =  0.38;
        public static final double FR_ENCODER_OFFSET_ROT = -0.01;
        public static final double BL_ENCODER_OFFSET_ROT =  0.325;
        public static final double BR_ENCODER_OFFSET_ROT =  0.21;

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE   = Math.PI * WHEEL_DIAMETER_METERS;

        public static final double MAX_SPEED_MPS         = 3.5;
        public static final double MAX_ANGULAR_SPEED_RPS = 3.0 * Math.PI;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d( WHEEL_BASE_METERS / 2,  TRACK_WIDTH_METERS / 2),
            new Translation2d( WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2,  TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)
        );

        public static final double STEER_kP = 50.0;
        public static final double STEER_kI =  0.0;
        public static final double STEER_kD =  0.5;

        // FIX: kV was 0.12 which is way too low — motor was never reaching target velocity.
        // Correct value: 12V / (6000 RPM / 6.12 gear ratio / 60) = 0.735 V·s/rot
        // kS accounts for static friction to get the motor moving at all.
        public static final double DRIVE_kP = 0.1;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double DRIVE_kV = 0.735;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2*3.14;
        public static final double DRIVE_kS = 0.15;

        public static final String CAN_BUS_NAME = "";
    }

    public static final class OIConstants {
        public static final int    DRIVER_CONTROLLER_PORT   = 0;
        public static final int    OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.08;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_ROLLER_ID = 26;
        public static final int INTAKE_PIVOT_ID  = 25;

        public static final double PIVOT_kP = 2.0;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.1;
        public static final double PIVOT_kG = 0.3;

        public static final double PIVOT_DEPLOYED_ROT  = 10.0;
        public static final double PIVOT_RETRACTED_ROT =  0.0;
        public static final double PIVOT_GEAR_RATIO    = 25.0;

        public static final double ROLLER_INTAKE_SPEED  =  0.6;
        public static final double ROLLER_OUTTAKE_SPEED = -0.6;

        public static final double ROLLER_SUPPLY_LIMIT = 40;
        public static final double PIVOT_SUPPLY_LIMIT  = 40;
        public static final double PIVOT_STATOR_LIMIT  = 60;
    }

    public static final class FeederConstants {
        public static final int FEEDER_ID = 27;

        public static final double FEEDER_SPEED         = -0.2;
        public static final double FEEDER_REVERSE_SPEED = 0.2;

        public static final double FEEDER_SUPPLY_LIMIT = 40;
        public static final double FEEDER_STATOR_LIMIT = 60;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_1_ID = 21;
        public static final int SHOOTER_2_ID = 22;
        public static final int SHOOTER_3_ID = 23;
        public static final int SHOOTER_4_ID = 24;

        public static final double SHOOTER_kP = 0.55;
        public static final double SHOOTER_kI = 0.0;
        public static final double SHOOTER_kD = 0.01;
        public static final double SHOOTER_kV = 0.0957;
        public static final double SHOOTER_kS = 0.0;

        public static final double SHOOT_VELOCITY_RPS   =  47.0;
        public static final double REVERSE_VELOCITY_RPS = -53.0;
        public static final double SHOOTER_4_INTAKE_RPS =  100.0;

        public static final double SHOOTER_SUPPLY_LIMIT = 60;
        public static final double SHOOTER_STATOR_LIMIT = 80;

        public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    

        // ─── Distance → RPS Lookup Table ──────────────────────────────────────
        // Key   = distance from target in METERS  (measured from Limelight)
        // Value = shooter flywheel speed in RPS
        // ⚠️  Tune these values by shooting from known distances and recording
        //     what RPS consistently scores. Add/remove rows as needed.
        public static final InterpolatingDoubleTreeMap SHOOTER_MAP =
            new InterpolatingDoubleTreeMap();

        static {
            SHOOTER_MAP.put(0.0, 40.0);
            SHOOTER_MAP.put(1.18, 54.5);
            SHOOTER_MAP.put(1.39, 54.0);
            SHOOTER_MAP.put(1.63, 56.5);
            SHOOTER_MAP.put(2.44, 65.0);
            SHOOTER_MAP.put(3.27, 69.89);
            SHOOTER_MAP.put(20.0,100.0);
        }
    }

    public static final class LimelightConstants {

        // ─── Network ──────────────────────────────────────────────────────────
        public static final String LIMELIGHT_NAME    = "limelight"; // match your LL web UI hostname
        public static final int    APRILTAG_PIPELINE = 0;           // pipeline index for April Tag mode

        // ─── Hub April Tag IDs ────────────────────────────────────────────────
        public static final int[] HUB_TAG_IDS = { 2, 9, 10, 11 };

        // ─── Hub Position (field-relative, WPILib Blue origin) ────────────────
        // Source: -120.236220 in, 22.950620 in, 0.375000 in
        public static final double HUB_X_METERS = -3.0540;
        public static final double HUB_Y_METERS =  0.5829;
        public static final double HUB_Z_METERS =  0.0095;

        // ─── Arc Zone ─────────────────────────────────────────────────────────
        // Robot must be inside this radius AND holding the button to auto-align
        public static final double ARC_RADIUS_METERS = 3.0;

        // ─── Camera Mounting (Limelight 4) ────────────────────────────────────
        // TODO: measure both values on your robot
        public static final double CAMERA_HEIGHT_METERS   = 0.50;  // lens height from floor (m)
        public static final double CAMERA_MOUNT_ANGLE_DEG = 20.0;  // upward tilt angle (deg)

        // ─── Tag Height ───────────────────────────────────────────────────────
        // 2025 Reefscape reef tags (9-11): centre at 29.875 in = 0.759 m
        public static final double TAG_HEIGHT_METERS = 0.759; // TODO: verify for your tags

        // ─── Alignment Target ─────────────────────────────────────────────────
        public static final double TARGET_DISTANCE_METERS = 1.0; // 1 m from tag face

        // ─── Rotation PID  (tx degrees → rad/s) ──────────────────────────────
        public static final double ALIGN_ROT_kP            = 0.04;
        public static final double ALIGN_ROT_kI            = 0.0;
        public static final double ALIGN_ROT_kD            = 0.002;
        public static final double ALIGN_ROT_TOLERANCE_DEG = 1.5;
        public static final double ALIGN_MAX_ROT_SPEED     = 1.5;  // rad/s

        // ─── Distance PID  (metres error → m/s) ──────────────────────────────
        public static final double ALIGN_DIST_kP          = 1.2;
        public static final double ALIGN_DIST_kI          = 0.0;
        public static final double ALIGN_DIST_kD          = 0.05;
        public static final double ALIGN_DIST_TOLERANCE_M = 0.05; // ±5 cm
        public static final double ALIGN_MAX_FWD_SPEED    = 1.5;  // m/s

        // ─── Settle ───────────────────────────────────────────────────────────
        // Both errors must stay in-tolerance for this many consecutive 20 ms loops
        public static final int ALIGN_SETTLE_LOOPS = 10; // = 200 ms
    }

    private Constants() {}
}

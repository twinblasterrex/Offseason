package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DrivetrainConstants {
	// Mechanical Constants
	// 0.5969 roughly
	public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
	public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
	public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
	public static final double WHEEL_DIAMETER = 0.0968375;
	public static final double GEAR_RATIO = 8.16;

	// PORT #s and OFFSETS
	public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
	public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
	public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 15;
	public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.30);

	public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
	public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
	public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
	public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.62);

	public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
	public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
	public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
	public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.22);

	public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
	public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
	public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
	public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.63);

	public final static double MAX_VOLTAGE = 12;// 12

	// Theoritcal speed cap XY
//		public static final double MAX_VELOCITY_METERS_PER_SECOND = 11000.0 / 60.0 *
//				SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
//				SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
	public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.8;
	// Theorital rot speed cap
	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

	// Max speed is scary
	public static final double SPEED_DEFFULT = .8;
	public static final double ANGULAR_SPEED_DEFAULT = 0.65;
	public static double SPEED_LIMIT = SPEED_DEFFULT;
	public static double ANGULAR_SPEED_LIMIT = ANGULAR_SPEED_DEFAULT;
}

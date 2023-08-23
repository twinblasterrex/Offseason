package frc.robot.subsystems.Drivetrain;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase {
	private static DrivetrainSubsystem INSTANCE;

	private DrivetrainMode mode;

	private SwerveModulePosition frontLPosition;
	private SwerveModulePosition frontRPosition;
	private SwerveModulePosition backLPosition;
	private SwerveModulePosition backRPosition;

	private SwerveModuleState frontLState;
	private SwerveModuleState frontRState;
	private SwerveModuleState backLState;
	private SwerveModuleState backRState;

	private ChassisSpeeds currentSpeeds;

	private ChassisSpeeds targetTeleopSpeeds;
	private ChassisSpeeds targetAutoSpeeds;

	private final SwerveModuleIO frontLeft;
	private final SwerveModuleIO frontRight;
	private final SwerveModuleIO backLeft;
	private final SwerveModuleIO backRight;

	private final ModuleDetails frontLeftDetails;
	private final ModuleDetails frontRightDetails;
	private final ModuleDetails backLeftDetails;
	private final ModuleDetails backRightDetails;

	private final SwerveDriveKinematics kinematics;
//	private final SwerveDriveOdometry odometry;
//
//	private final Navx navx;

	private DrivetrainSubsystem() {

		frontLeftDetails = new ModuleDetails(drivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,drivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,drivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,drivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
		frontRightDetails = new ModuleDetails(drivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,drivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,drivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,drivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
		backLeftDetails = new ModuleDetails(drivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,drivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,drivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,drivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
		backRightDetails = new ModuleDetails(drivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, drivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR, drivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER, drivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

		if (Robot.isSimulation())
		{
			frontLeft = new SwerveModuleSim(frontLeftDetails);
			frontRight = new SwerveModuleSim(frontRightDetails);
			backLeft = new SwerveModuleSim(backLeftDetails);
			backRight = new SwerveModuleSim(backRightDetails);
		}else {
			frontLeft = new SwerveModuleReal(frontLeftDetails);
			frontRight = new SwerveModuleReal(frontRightDetails);
			backLeft = new SwerveModuleReal(backLeftDetails);
			backRight = new SwerveModuleReal(backRightDetails);
		}

		kinematics = DrivetrainConstants.KINEMATICS;
	}

	@Override
	public void periodic() {
		if (Robot.isSimulation())
		{

		}
		if (Robot.isReal())
		{

		}

	}

	public DrivetrainMode getMode() {
		return mode;
	}

	public void setMode(DrivetrainMode Mode) {
		mode = Mode;
	}

	/**
	 * @param x this is the x input
	 * @param y this is the y input
	 * @param z rotate input
	 * This should get values that have already been altered and changed on a scale of -Max speed to Max speed
	 */
	public void setTargetTeleopSpeeds(double x, double y, double z)
	{
		targetTeleopSpeeds = new ChassisSpeeds(x,y,z);
	}

	public void setTargetAutoSpeeds(double x, double y, double z)
	{
		targetAutoSpeeds = new ChassisSpeeds(x,y,z);
	}

	public void setTargetAutoSpeeds(ChassisSpeeds speeds)
	{
		targetAutoSpeeds = speeds;
	}
	public void setTargetAutoSpeeds(SwerveModuleState[] states)
	{
		targetAutoSpeeds = kinematics.toChassisSpeeds(states);
	}
	public ChassisSpeeds getTargetTeleopSpeeds() {
		return targetTeleopSpeeds;
	}

	public void setTargetTeleopSpeeds(ChassisSpeeds mTargetTeleopSpeeds) {
		targetTeleopSpeeds = mTargetTeleopSpeeds;
	}

	public ChassisSpeeds getTargetAutoSpeeds() {
		return targetAutoSpeeds;
	}

	public static DrivetrainSubsystem getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new DrivetrainSubsystem();
		}
		return INSTANCE;
	}
}


final class drivetrainConstants
{
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

}
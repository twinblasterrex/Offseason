package frc.robot.subsystems.Drivetrain;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NavxSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {

	private static DrivetrainSubsystem INSTANCE;
	//
	//
	private final ModuleIO front_left;
	private final ModuleIO front_right;
	private final ModuleIO back_left;
	private final ModuleIO back_right;
	//
	//
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveModuleState[] targetStates; // This is the current applied states
	private SwerveModuleState[] currentStates; // These are the states according to the sensors on the swerve modules
	private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}; // Each swerve module class will manage the module positions and it will be combined in the update drive function
	//
	//
	private ChassisSpeeds autoDrive = new ChassisSpeeds();
	private ChassisSpeeds teleopDrive = new ChassisSpeeds();
	private ChassisSpeeds visionDrive = new ChassisSpeeds();
	//
	private DrivetrainMode drivetrainMode;
	//
	//
	private NavxSim gyro_sim;
	private AHRS gyro;

	private DrivetrainSubsystem() {
		drivetrainMode = DrivetrainMode.kFullcontrollor;

		kinematics = CONSTANTS.KINEMATICS;

		switch (Constants.STATE) {
			case isReal:
				front_left = new ModuleReal(6, 5, 15, 269.30);
				front_right = new ModuleReal(7, 8, 13, 238.60);
				back_left = new ModuleReal(1, 2, 16, 168.22);
				back_right = new ModuleReal(3, 4, 14, 79.63);

				gyro = new AHRS();
				break;
			default:
				front_left = new ModuleSim(6, 5, 15, 269.30);
				front_right = new ModuleSim(7, 8, 13, 238.60);
				back_left = new ModuleSim(1, 2, 16, 168.22);
				back_right = new ModuleSim(3, 4, 14, 79.63);

				gyro_sim = new NavxSim();
				break;
		}


	}

	@SuppressWarnings("WeakerAccess")
	public static DrivetrainSubsystem getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new DrivetrainSubsystem();
		}
		return INSTANCE;
	}


	@Override
	public void periodic() {
		ChassisSpeeds speeds; // This is the speeds that we will set
		speeds = new ChassisSpeeds();
		switch (drivetrainMode) {
			case kFullcontrollor:
				speeds = teleopDrive;
				break;
			case kAutoControl:
				speeds = autoDrive;
				break;
			case kVisionControl:
				speeds = visionDrive;
				break;
		}

		SwerveModuleState[] states =
				kinematics.toSwerveModuleStates(
						ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation())
				);

		// The angle is in Rotation
		front_left.setModuleState(states[0]);
		front_right.setModuleState(states[1]);
		back_left.setModuleState(states[2]);
		back_right.setModuleState(states[3]);

		modulePositions[0] = front_left.getModulePosition();
		modulePositions[1] = front_right.getModulePosition();
		modulePositions[2] = back_left.getModulePosition();
		modulePositions[3] = back_right.getModulePosition();

		front_left.periodic();
		front_right.periodic();
		back_left.periodic();
		back_right.periodic();


		Logger.getInstance().recordOutput("Drivetrain/GyroAngle_DEGREES", getRotation().getDegrees());
		Logger.getInstance().recordOutput("Drivetrain/Pose", getPose());

		Logger.getInstance().recordOutput("Drivetrain/ModulePositions", new double[] { // The unit is radians
				states[0].angle.getRadians(), states[0].speedMetersPerSecond,
				states[1].angle.getRadians(), states[1].speedMetersPerSecond,
				states[2].angle.getRadians(), states[2].speedMetersPerSecond,
				states[3].angle.getRadians(), states[3].speedMetersPerSecond
		}); // This is for the visualization feature
	}

	public Rotation2d getRotation() // This is the rotation of the robot based on gyro
	{
		switch (Constants.STATE) {
			case isReal:
				return gyro.getRotation2d();
			case isSim:
				return new Rotation2d(Math.toRadians(gyro_sim.getAngle()));
		}
		return null;
	}

	public ChassisSpeeds getVisionDrive() {
		return visionDrive;
	}

	public void setVisionDrive(ChassisSpeeds mVisionDrive) {
		visionDrive = mVisionDrive;
	}

	public ChassisSpeeds getTeleopDrive() {
		return teleopDrive;
	}

	public void setTeleopDrive(ChassisSpeeds mTeleopDrive) {
		teleopDrive = mTeleopDrive;
	}

	public ChassisSpeeds getAutoDrive() {
		return autoDrive;
	}

	public void setAutoDrive(ChassisSpeeds mAutoDrive) {
		autoDrive = mAutoDrive;
	}

	public Pose2d getPose()
	{
		return new Pose2d();
	}

	static class CONSTANTS {
		// This is for a frame perimeter of 30x30 inches these units are in meters
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
		public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0));
		public static double MAX_SPEED = 3.68; // This is around 12 feet per second

		public static final class frontLeft {
			public final static int STEER_ID = 5;
			public final static int DRIVE_ID = 6;

			public final static int CAN_CODER_ID = 15;
			public final static double CAN_CODER_OFFSET = 269.30;
		}

		public static final class frontRight {
			public final static int STEER_ID = 7;
			public final static int DRIVE_ID = 8;

			public final static int CAN_CODER_ID = 13;
			public final static double CAN_CODER_OFFSET = 238.62;
		}

		public static final class backLeft {
			public final static int STEER_ID = 1;
			public final static int DRIVE_ID = 2;

			public final static int CAN_CODER_ID = 16;
			public final static double CAN_CODER_OFFSET = 168.22;
		}

		public static final class backRight {
			public final static int STEER_ID = 3;
			public final static int DRIVE_ID = 4;

			public final static int CAN_CODER_ID = 14;
			public final static double CAN_CODER_OFFSET = 79.63;
		}
	}
}
package frc.robot.subsystems.Drivetrain;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.navx.Navx;
import frc.lib.navx.NavxReal;
import frc.lib.navx.NavxSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.Drivetrain.DrivetrainConstants.*;

public class DrivetrainSubsystem extends SubsystemBase {
	private static DrivetrainSubsystem INSTANCE;

	private DrivetrainMode mode;

	private SwerveModulePosition frontLeftPosition;
	private SwerveModulePosition frontRightPosition;
	private SwerveModulePosition backLeftPosition;
	private SwerveModulePosition backRightPosition;

	private SwerveModuleState frontLeftState;
	private SwerveModuleState frontRightState;
	private SwerveModuleState backLeftState;
	private SwerveModuleState backRightState;

	private ChassisSpeeds currentSpeeds;

	private ChassisSpeeds targetTeleopSpeeds;
	private ChassisSpeeds targetAutoSpeeds;
	private ChassisSpeeds targetAutoAlineSpeeds;
	private ChassisSpeeds targetTeleopAutoAlineSpeeds;

	private final SwerveModuleIO frontLeft;
	private final SwerveModuleIO frontRight;
	private final SwerveModuleIO backLeft;
	private final SwerveModuleIO backRight;

	private final ModuleDetails frontLeftDetails;
	private final ModuleDetails frontRightDetails;
	private final ModuleDetails backLeftDetails;
	private final ModuleDetails backRightDetails;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;

	private final Navx navx;

	private DrivetrainSubsystem() {

		frontLeftDetails = new ModuleDetails(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET, ModuleEnum.frontLeft);
		frontRightDetails = new ModuleDetails(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET, ModuleEnum.frontRight);
		backLeftDetails = new ModuleDetails(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET, ModuleEnum.backLeft);
		backRightDetails = new ModuleDetails(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET, ModuleEnum.backRight);

		if (Robot.isSimulation())
		{
			frontLeft = new SwerveModuleSim(frontLeftDetails);
			frontRight = new SwerveModuleSim(frontRightDetails);
			backLeft = new SwerveModuleSim(backLeftDetails);
			backRight = new SwerveModuleSim(backRightDetails);

			navx = new NavxSim();
		}else {
			frontLeft = new SwerveModuleReal(frontLeftDetails);
			frontRight = new SwerveModuleReal(frontRightDetails);
			backLeft = new SwerveModuleReal(backLeftDetails);
			backRight = new SwerveModuleReal(backRightDetails);

			navx = new NavxReal();
		}

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		frontLeftState = new SwerveModuleState();
		frontRightState = new SwerveModuleState();
		backLeftState = new SwerveModuleState();
		backRightState = new SwerveModuleState();

		currentSpeeds = new ChassisSpeeds();

		targetAutoSpeeds = new ChassisSpeeds();
		targetTeleopSpeeds = new ChassisSpeeds();
		targetTeleopAutoAlineSpeeds = new ChassisSpeeds();
		targetAutoAlineSpeeds = new ChassisSpeeds();

		kinematics = KINEMATICS;
		odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition});
		mode = DrivetrainMode.teleop;
	}

	@Override
	public void periodic() {

		ChassisSpeeds targetSpeed = new ChassisSpeeds();
		switch (mode)
		{
			case teleop:
				targetSpeed = targetTeleopSpeeds;
				break;
			case teleop_autoAline:
				targetSpeed = targetTeleopAutoAlineSpeeds;
				break;
			case auto:
				targetSpeed = targetAutoSpeeds;
				break;
			case auto_autoAline:
				targetSpeed = targetAutoAlineSpeeds;
				break;
		}

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeed, navx.getRotation2d())
		);

		states[0] = SwerveModuleState.optimize(states[0], frontLeft.getRotation2d());
		states[1] = SwerveModuleState.optimize(states[1], frontRight.getRotation2d());
		states[2] = SwerveModuleState.optimize(states[2], backLeft.getRotation2d());
		states[3] = SwerveModuleState.optimize(states[3], backRight.getRotation2d());

		frontLeft.setSwerveModuleState(states[0]);
		frontRight.setSwerveModuleState(states[1]);
		backLeft.setSwerveModuleState(states[2]);
		backRight.setSwerveModuleState(states[3]);

		// Commented out for testing purposes
		frontLeftState = states[0];
		frontRightState = states[1];
		backLeftState = states[2];
		backRightState = states[3];

		currentSpeeds = targetSpeed;

		if (Robot.isSimulation())
		{
			// gyro update
			double gyroRate = currentSpeeds.omegaRadiansPerSecond;
			navx.setRate(gyroRate);

			navx.update(Robot.defaultPeriodSecs);
		}

		frontLeft.periodic();
		frontRight.periodic();
		backLeft.periodic();
		backRight.periodic();

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		odometry.update(getRotation(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition});

		Logger.getInstance().recordOutput("states", states);
		Logger.getInstance().recordOutput("rotation" ,getRotation().getRadians());
		Logger.getInstance().recordOutput("pose", getPose());

	}

	public void resetOdometry(Pose2d newPose)
	{
		navx.zeroYaw();

		frontLeft.resetModulePositions();
		frontRight.resetModulePositions();
		backLeft.resetModulePositions();
		backRight.resetModulePositions();

		frontLeftPosition = frontLeft.getModulePosition();
		frontRightPosition = frontRight.getModulePosition();
		backLeftPosition = backLeft.getModulePosition();
		backRightPosition = backRight.getModulePosition();

		odometry.resetPosition(getRotation(),new SwerveModulePosition[] {frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition}, newPose);
	}
	public DrivetrainMode getMode() {
		return mode;
	}

	public Rotation2d getRotation()
	{
		return navx.getRotation2d();
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

	public Pose2d getPose()
	{
		return odometry.getPoseMeters();
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

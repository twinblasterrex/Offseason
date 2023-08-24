package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleReal implements SwerveModuleIO {

	private final CANSparkMax drive;
	private final CANSparkMax steer;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder steerEncoder;

	private final PIDController driveController;
	private final PIDController steerController;

	private CANCoder encoder;
	private double steerOffset;

	private String moduleName = "";

	private SwerveModuleState state;
	private SwerveModulePosition position;

	double driveVoltage;
	double steerVoltage;

	double driveSetpoint;
	double steerSetpoint;

	public SwerveModuleReal(ModuleDetails details)
	{
		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		drive.restoreFactoryDefaults();
		steer.restoreFactoryDefaults();

		drive.clearFaults();
		steer.clearFaults();

		drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steer.setIdleMode(CANSparkMax.IdleMode.kBrake);

		drive.setSmartCurrentLimit(40);
		steer.setSmartCurrentLimit(20);

		driveEncoder = drive.getEncoder();
		steerEncoder = steer.getEncoder();

		driveController = new PIDController(0,0,0);
		steerController = new PIDController(0,0,0);

		encoder = new CANCoder(details.encoderID);

		CANCoderConfiguration configuration = new CANCoderConfiguration();
		configuration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
		configuration.magnetOffsetDegrees = details.encoderOffset;
		encoder.clearStickyFaults();
		encoder.configAllSettings(configuration);

		state = new SwerveModuleState();
		position = new SwerveModulePosition();

		driveVoltage = 0;
		steerVoltage = 0;

		driveSetpoint = 0;
		steerSetpoint = 0;

		switch (details.module)
		{
			case frontLeft:
				moduleName = "frontLeft";
				break;
			case frontRight:
				moduleName = "frontRight";
				break;
			case backLeft:
				moduleName = "backLeft";
				break;
			case backRight:
				moduleName = "backRight";
				break;
		}
	}

	@Override
	public void periodic() {
		if (RobotState.isEnabled())
		{
			driveSetpoint = (state.speedMetersPerSecond / 3.6)* 5675;

			driveVoltage = driveController.calculate(
					driveEncoder.getVelocity(),
					driveSetpoint
			);

			driveVoltage = MoreMath.minMax(driveVoltage, -12,12);

			drive.setVoltage(driveVoltage);
		}

		if (RobotState.isDisabled())
		{
			driveSetpoint = 0;
			steerSetpoint = 0;

			driveController.reset();
			steerController.reset();
		}

		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/DriveMotorTemp", drive.getMotorTemperature());
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/DriveMotorVelocity", drive.getEncoder().getVelocity());
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/DriveVoltageApplied",driveVoltage);
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/DriveSetpoint",driveSetpoint);

		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/SteerMotorTemp",steer.getMotorTemperature());
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/SteerEncoderPosition",getRotation2d().getRadians());
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/SteerVoltageApplied",steerVoltage);
		Logger.getInstance().recordOutput("SwerveModule/" + moduleName + "/SteerSetpoint",steerSetpoint);
	}

	@Override
	public void resetModulePositions() {
		driveEncoder.setPosition(0);
		position = new SwerveModulePosition();
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
	}

	@Override
	public double getRPM() {
		return driveEncoder.getVelocity();
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return position;
	}

	@Override
	public void setSwerveModuleState(SwerveModuleState state) {
		this.state = state;
	}
}

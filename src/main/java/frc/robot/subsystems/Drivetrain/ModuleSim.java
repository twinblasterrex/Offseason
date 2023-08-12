package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CANCoderSim;
import frc.lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class ModuleSim implements ModuleIO {

	private final int m_driveChannel;
	private final int m_steerChannel;
	private final int m_encoderChannel;
	private final double m_encoderOffset;
	private final CANSparkMax drive;
	private final CANSparkMax steer;
	private final Module module;
	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder steerEncoder;
	private final PIDController driveController;
	/**
	 * RPM control of wheel speed
	 * Reason:
	 * 1. It provides a more stable and consistent speed across all the wheels insuring that they are all moving at the same speed
	 * IMPORTANT INFORMATION:
	 * The controller will be controlling the voltage of the motor. The output is min maxed to ensure that it does not go beyond 12 volts
	 * This controller is not continues and a max and min i value must be set
	 * The setpoint point should be the rpm after the gear ratio is accounted for
	 * The calculate point is just going to be the velocity output of the encoder
	 */
	private final PIDController steerController;
	private final CANCoderSim encoder;
	private SwerveModuleState state;
	private String moduleName;
	private double driveSetpoint = 0; // This is a value in the unites of RPM
	private double steerSetpoint = 0; // This is a value in the unites of Radians

	public ModuleSim(
			int driveChannel,
			int steerChannel,
			int encoderChannel,
			double encoderOffset,
			Module module
	) {

		m_driveChannel = driveChannel;
		m_steerChannel = steerChannel;
		m_encoderChannel = encoderChannel;
		m_encoderOffset = encoderOffset;

		this.module = module;

		drive = new CANSparkMax(driveChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(steerChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

		encoder = new CANCoderSim(encoderChannel, encoderOffset, DCMotor.getNEO(1).withReduction((150 / 7)));

		REVPhysicsSim.getInstance().addSparkMax(drive, DCMotor.getNEO(1).withReduction(6.75)); // The gear ratio is 6.74/1
//		REVPhysicsSim.getInstance().addSparkMax(drive, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(steer, DCMotor.getNEO(1).withReduction((150 / 7))); // The gear ratio is 150/7 : 1

		driveEncoder = drive.getEncoder();
		steerEncoder = steer.getEncoder();

		driveController = new PIDController(.01, 0, 0);
		steerController = new PIDController(.5, 0, 0);

		driveController.setTolerance(20);
//		driveController.setIntegratorRange(-12, 12);

		driveController.enableContinuousInput(-180, 180);

		moduleName = "";
		switch (module) {
			case FL:
				moduleName = "Drivetrain/FrontLeft/";
				break;
			case FR:
				moduleName = "Drivetrain/FrontRight/";
				break;
			case BL:
				moduleName = "Drivetrain/BackLeft/";
				break;
			case BR:
				moduleName = "Drivetrain/BackRight/";
				break;
		}
	}

	@Override
	public void periodic() {
		driveSetpoint = state.speedMetersPerSecond / DrivetrainSubsystem.CONSTANTS.MAX_SPEED;
		driveSetpoint = driveSetpoint * 12;

		System.out.println(driveSetpoint);

		drive.setVoltage(driveSetpoint);


		steerSetpoint = state.angle.getRadians();
		steer.setVoltage(steerController.calculate(
				encoder.getRotation().getDegrees(),
				steerSetpoint
		));
	}

	@Override
	public void log() {
		Logger.getInstance().recordOutput(moduleName + "RPM", getRPM());
		Logger.getInstance().recordOutput(moduleName + "steerPosition", getSteerPosition());
		Logger.getInstance().recordOutput(moduleName + "drive_temp", getDriveMotorTemp());
		Logger.getInstance().recordOutput(moduleName + "steer_temp", getSteerMotorTemp());
		Logger.getInstance().recordOutput(moduleName + "RPM_SETPOINT", driveSetpoint);
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(driveEncoder.getPosition() * (4 * Math.PI), encoder.getRotation()); // 4 is the diameter of the wheel and we are multiplying the wheel position by the circumference
	}

	@Override
	public void setModuleState(SwerveModuleState state) {
		this.state = state;
	}

	@Override
	public SwerveModuleState getTargetState() { // This returns the target state
		return state;
	}

	@Override
	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(MoreMath.toMeters((driveEncoder.getVelocity() / 60) * (4 * Math.PI) / 12), encoder.getRotation());
	}

	@Override
	public double getDriveMotorTemp() {
		return drive.getMotorTemperature();
	}

	@Override
	public double getSteerMotorTemp() {
		return steer.getMotorTemperature();
	}

	@Override
	public double getDriveMotorAppliedVoltage() {
		return drive.getAppliedOutput();
	}

	@Override
	public double steerMotorAppliedVoltage() {
		return steer.getAppliedOutput();
	}

	@Override
	public double getSteerSetpoint() {
		return steerSetpoint;
	}

	@Override
	public double getSteerPosition() {
		return encoder.getRotation().getRadians();
	}

	@Override
	public double getRawEncoderOutput() {
		return encoder.getRotation().getRadians();
	}

	@Override
	public double getRPM() {
		return driveEncoder.getVelocity();
	}
}

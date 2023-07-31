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

public class ModuleSim implements ModuleIO {

	private final int m_driveChannel;
	private final int m_steerChannel;
	private final int m_encoderChannel;
	private final double m_encoderOffset;


	private final CANSparkMax drive;
	private final CANSparkMax steer;

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
	private double driveSetpoint = 0; // This is a value in the unites of RPM
	private double steerSetpoint = 0; // This is a value in the unites of Radians

	public ModuleSim(
			int driveChannel,
			int steerChannel,
			int encoderChannel,
			double encoderOffset
	) {

		m_driveChannel = driveChannel;
		m_steerChannel = steerChannel;
		m_encoderChannel = encoderChannel;
		m_encoderOffset = encoderOffset;

		drive = new CANSparkMax(driveChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(steerChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

		encoder = new CANCoderSim(encoderChannel, encoderOffset,DCMotor.getNEO(1).withReduction((150 / 7) / 1));

		REVPhysicsSim.getInstance().addSparkMax(drive, DCMotor.getNEO(1).withReduction(6.74 / 1)); // The gear ratio is 6.74/1
		REVPhysicsSim.getInstance().addSparkMax(steer, DCMotor.getNEO(1).withReduction((150 / 7) / 1)); // The gear ratio is 150/7 : 1

		driveEncoder = drive.getEncoder();
		steerEncoder = steer.getEncoder();

		driveController = new PIDController(0, 0, 0);
		steerController = new PIDController(0, 0, 0);

		driveController.setTolerance(20);
		driveController.setIntegratorRange(-12, 12);

		driveController.enableContinuousInput(-180, 180);
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition();
	}

	@Override
	public void setModuleState(SwerveModuleState state) {

	}

	@Override
	public SwerveModuleState getState() {
		return null;
	}

	@Override
	public double driveMotorTemp() {
		return 0;
	}

	@Override
	public double steerMotorTemp() {
		return 0;
	}

	@Override
	public double driveMotorAppliedVoltage() {
		return 0;
	}

	@Override
	public double steerMotorAppliedVoltage() {
		return 0;
	}

	@Override
	public double steerSetpoint() {
		return 0;
	}

	@Override
	public double steerPosition() {
		return 0;
	}

	@Override
	public double getRawEncoderOutput() {
		return 0;
	}

	@Override
	public double getRPM() {
		return 0;
	}

	@Override
	public void periodic() {

	}
}

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleReal implements ModuleIO {

	private final int driveChannel;
	private final int steerChannel;
	private final int encoderChannel;
	private final double encoderOffset;

	public ModuleReal(
			int driveChannel,
			int steerChannel,
			int encoderChannel,
			double encoderOffset
	) {

		this.driveChannel = driveChannel;
		this.steerChannel = steerChannel;
		this.encoderChannel = encoderChannel;
		this.encoderOffset = encoderOffset;
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition();
	}

	@Override
	public void setModuleState(SwerveModuleState state) {

	}

	@Override
	public SwerveModuleState getTargetState() {
		return null;
	}

	@Override
	public SwerveModuleState getCurrentState() {
		return null;
	}

	@Override
	public double getDriveMotorTemp() {
		return 0;
	}

	@Override
	public double getSteerMotorTemp() {
		return 0;
	}

	@Override
	public double getDriveMotorAppliedVoltage() {
		return 0;
	}

	@Override
	public double steerMotorAppliedVoltage() {
		return 0;
	}

	@Override
	public double getSteerSetpoint() {
		return 0;
	}

	@Override
	public double getSteerPosition() {
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

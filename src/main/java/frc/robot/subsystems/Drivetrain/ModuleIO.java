package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

	SwerveModulePosition getModulePosition();

	void setModuleState(SwerveModuleState state);// The unites should be in radians and in meters per second

	/**
	 * @return This returns the target state that the module is currently trying to achieve
	 */
	SwerveModuleState getTargetState();

	/**
	 * @return This is the state read from the sensors
	 */
	SwerveModuleState getCurrentState();

	double getDriveMotorTemp();

	double getSteerMotorTemp();

	double getDriveMotorAppliedVoltage();

	double steerMotorAppliedVoltage();

	double getSteerSetpoint();

	double getSteerPosition();

	double getRawEncoderOutput();

	double getRPM(); // This is the RPM of the drive motor

	void periodic();

	default void log() {

	}
}

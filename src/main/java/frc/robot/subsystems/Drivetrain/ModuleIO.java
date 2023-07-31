package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

	public SwerveModulePosition getModulePosition();

	public void setModuleState(SwerveModuleState state);// The unites should be in radians and in meters per second

	public SwerveModuleState getState();

	public double driveMotorTemp();

	public double steerMotorTemp();

	public double driveMotorAppliedVoltage();

	public double steerMotorAppliedVoltage();

	public double steerSetpoint();

	public double steerPosition();

	public double getRawEncoderOutput();

	public double getRPM(); // This is the RPM of the drive motor

	public void periodic();

	public default void log() {

	}
}

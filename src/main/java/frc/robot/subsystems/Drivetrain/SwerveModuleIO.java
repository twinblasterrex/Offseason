package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

	public default SwerveModulePosition getModulePosition() {return new SwerveModulePosition();}
	public default void setSwerveModuleState(SwerveModuleState state) {}

	public default Rotation2d getRotation2d() {return new Rotation2d();};
	public default double getRPM() {return 0;}

	public default void periodic() {}

	public void resetModulePositions();
}

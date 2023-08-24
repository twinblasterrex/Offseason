package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

public class SwerveModuleReal implements SwerveModuleIO {
	private CANSparkMax drive;
	private CANSparkMax steer;

	private CANCoder encoder;
	private double steerOffset;

	public SwerveModuleReal(ModuleDetails details)
	{

	}
}

package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import frc.lib.CanCoderSim;

public class SwerveModuleSim implements SwerveModuleIO {
	private CANSparkMax drive;
	private CANSparkMax steer;

	private CanCoderSim encoder;
	private double steerOffset;

	public SwerveModuleSim(ModuleDetails details)
	{

	}

}

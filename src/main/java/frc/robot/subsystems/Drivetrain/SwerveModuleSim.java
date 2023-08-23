package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import frc.lib.CANCoderSim;

public class SwerveModuleSim implements SwerveModuleIO {
	private CANSparkMax drive;
	private CANSparkMax steer;

	private CANCoderSim encoder;
	private double steerOffset;

	public SwerveModuleSim(ModuleDetails details)
	{

	}

}

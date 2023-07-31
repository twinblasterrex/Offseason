package frc.robot.sim;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimSubsystem extends SubsystemBase {
	private static SimSubsystem INSTANCE;

	private SimSubsystem() {

	}

	@SuppressWarnings("WeakerAccess")
	public static SimSubsystem getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new SimSubsystem();
		}
		return INSTANCE;
	}
}


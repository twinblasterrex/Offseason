package frc.robot.sim;


import com.revrobotics.REVPhysicsSim;
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

	@Override
	public void periodic() {

		REVPhysicsSim.getInstance().run();
	}
}


package frc.robot;

import frc.robot.util.RobotStateENUE;

import static frc.robot.util.RobotStateENUE.isReal;
import static frc.robot.util.RobotStateENUE.isSim;

public final class Constants {

	public static final RobotStateENUE STATE = getState();

	public static RobotStateENUE getState() {
		if (Robot.isSimulation()) {
			return isSim;
		}

		return isReal;
	}
}

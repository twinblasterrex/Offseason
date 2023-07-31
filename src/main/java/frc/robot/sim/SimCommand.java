package frc.robot.sim;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class SimCommand extends CommandBase {
	private final SimSubsystem simSubsystem = SimSubsystem.getInstance();

	public SimCommand() {

		addRequirements(this.simSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}

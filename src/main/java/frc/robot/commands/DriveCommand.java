package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Deadband;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;


public class DriveCommand extends CommandBase {
	private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();

    private final PS4Controller controller = new PS4Controller(0);

    private final Deadband xyDeadband = new Deadband(.03,0);
    private final Deadband rotDeadband = new Deadband(.03,0);

	public DriveCommand() {
		addRequirements(this.drivetrainSubsystem);
	}

	@Override
	public void execute() {

        double x = xyDeadband.apply(-controller.getRawAxis(1));
        double y = xyDeadband.apply(-controller.getRawAxis(0));
        double theta = rotDeadband.apply(-controller.getRawAxis(2));

        x = Math.pow(x,3);
        y = Math.pow(y,3);
        theta = Math.pow(theta,3);

        x = x * 3.6;
        y = y * 3.6;
        theta = theta * 3.6;


        drivetrainSubsystem.setTargetTeleopSpeeds(
                x,
                y,
                theta
        );
	}

    @Override
    public void initialize() {

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

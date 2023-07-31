package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;


public class DefaultDriveCommand extends CommandBase {
	private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
	private final XboxController controller = new XboxController(0);
	public DefaultDriveCommand() {

		addRequirements(this.drivetrainSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		drivetrainSubsystem.setTeleopDrive(new ChassisSpeeds(controller.getLeftY(), controller.getLeftX(), controller.getRightX()));
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

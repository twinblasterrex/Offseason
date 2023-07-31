package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Deadband;
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

		Deadband x, y, z;
		x = new Deadband(.05, 0);
		y = new Deadband(.05, 0);
		z = new Deadband(.05, 0);

		double xInput, yInput, zInput;
		xInput = -controller.getLeftY();
		yInput = -controller.getLeftX();
		zInput = controller.getRightX();

		xInput = x.apply(xInput);
		yInput = y.apply(yInput);
		zInput = z.apply(zInput);

		xInput = xInput * DrivetrainSubsystem.CONSTANTS.MAX_SPEED;
		yInput = yInput * DrivetrainSubsystem.CONSTANTS.MAX_SPEED;
		zInput = zInput * DrivetrainSubsystem.CONSTANTS.MAX_TURNING_SPEED;

		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		zInput = Math.pow(zInput, 3);


		drivetrainSubsystem.setTeleopDrive(new ChassisSpeeds(xInput, yInput, zInput));

		if (controller.getRightTriggerAxis() > .75) {
			drivetrainSubsystem.resetGyro();
		}
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

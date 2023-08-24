package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainMode;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.HashMap;

public class Auto {
	private final SwerveAutoBuilder autoBuilder;
	private final HashMap<String, Command> eventMap = new HashMap<>();
	private final PathConstraints constraints;

	public Auto()
	{
		constraints = new PathConstraints(3.6,3.6);
		autoBuilder = new SwerveAutoBuilder(
				DrivetrainSubsystem.getInstance()::getPose,
				DrivetrainSubsystem.getInstance()::resetOdometry,
				DrivetrainConstants.KINEMATICS,
				new PIDConstants(0,0,0),
				new PIDConstants(0,0,0),
				DrivetrainSubsystem.getInstance()::setTargetAutoSpeeds,
				this.eventMap,
				true,
				DrivetrainSubsystem.getInstance()
		);
	}

	public Command genAuto(String autoName)
	{
		return new SequentialCommandGroup(
				new InstantCommand(() -> DrivetrainSubsystem.getInstance().setMode(DrivetrainMode.auto)),
				autoBuilder.fullAuto(
						PathPlanner.loadPath(autoName, constraints)
				),
				new InstantCommand(() -> DrivetrainSubsystem.getInstance().setMode(DrivetrainMode.teleop))
		);
	}
}

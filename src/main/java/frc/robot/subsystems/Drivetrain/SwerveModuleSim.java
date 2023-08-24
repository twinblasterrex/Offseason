package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CanCoderSim;
import frc.lib.MoreMath;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleSim implements SwerveModuleIO {
	private CANSparkMax drive;
	private CANSparkMax steer;

	private PIDController driveController;
	private PIDController steerControllor;

	private CanCoderSim encoder;
	private double steerOffset;

	private final RelativeEncoder driveEncoder;

	private SwerveModuleState targetState;
	private SwerveModulePosition position;

	public SwerveModuleSim(ModuleDetails details)
	{
		drive = new CANSparkMax(details.driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		REVPhysicsSim.getInstance().addSparkMax(drive, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(steer, DCMotor.getNEO(1));

		driveEncoder = drive.getEncoder();

		encoder = new CanCoderSim(details.encoderID, details.encoderOffset);

		targetState = new SwerveModuleState();

		position = new SwerveModulePosition();
		driveController = new PIDController(.0001,.004,0);
		steerControllor = new PIDController(10,0,0);
		steerControllor.setTolerance(.1);
		steerControllor.enableContinuousInput(-Math.PI,Math.PI);
	}

	@Override
	public void periodic() {
		double voltage =
		MoreMath.minMax(
				driveController.calculate(driveEncoder.getVelocity(),(targetState.speedMetersPerSecond / 3.6) * 5675),
				-1,
				1
		);
		drive.setVoltage(voltage);
		Logger.getInstance().recordOutput("voltage", voltage);
		Logger.getInstance().recordOutput("targetSpeed",(targetState.speedMetersPerSecond / 3.6) * 5675);
		Logger.getInstance().recordOutput("currentSpeed", driveEncoder.getVelocity());


		double steerVoltage =
		MoreMath.minMax(
				steerControllor.calculate(encoder.getRot().getRadians(),targetState.angle.getRadians()),
				-3 * Math.PI, 3 * Math.PI
				);

//		Logger.getInstance().recordOutput("atSetpoint", steerControllor.atSetpoint());
//		Logger.getInstance().recordOutput("targetAngle", targetState.angle.getRadians());
//		Logger.getInstance().recordOutput("currentAngle", encoder.getRot().getRadians());
//		Logger.getInstance().recordOutput("voltage", steerVoltage);

		encoder.setRate(steerVoltage);
		encoder.update(Robot.defaultPeriodSecs);
		position = new SwerveModulePosition((driveEncoder.getPosition() / 8.14) / 3, getRotation2d());
	}

	@Override
	public void resetModulePositions() {
		driveEncoder.setPosition(0);
		position = new SwerveModulePosition();
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return position;
	}

	@Override
	public void setSwerveModuleState(SwerveModuleState state) {
		targetState = state;
	}

	@Override
	public Rotation2d getRotation2d() {
		return encoder.getRot();
	}

	@Override
	public double getRPM() {
		return driveEncoder.getVelocity();
	}

}

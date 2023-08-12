package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CANCoderSim {

	private final int id;
	private final double offset;
	private final DCMotorSim motor;
	private double rate;
	private Rotation2d rotation;

	public CANCoderSim(int id, double offset, DCMotor motor) {
		this.motor = new DCMotorSim(motor, 1, 1);
		rate = 0;
		rotation = new Rotation2d();
		this.id = id;
		this.offset = offset;
	}

	public void setVoltage(double voltage) {
		motor.setInputVoltage(voltage);
	}

	public void update(double period) {
		setRate(motor.getAngularVelocityRadPerSec());

		rotation = new Rotation2d(
				rotation.getRadians() + (rate * period)
		);
	}

	public Rotation2d getRotation() {
		return rotation;
	}

	public void setRotation(Rotation2d mRotation) {
		rotation = mRotation;
	}

	public double getRate() {
		return rate;
	}

	/**
	 * @param mRate rate is in radians per second
	 */
	public void setRate(double mRate) {
		rate = mRate;
	}

	public double getOffset() {
		return offset;
	}

	public int getId() {
		return id;
	}
}

package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavxSim {
	private Rotation2d rotation = new Rotation2d();
	private double rate = 0;

	public NavxSim() {
	}

	public void update(double updateTime) {
		double currentAngle = rotation.getRadians();

		rotation = new Rotation2d(currentAngle + (updateTime * rate));
	}

	public Rotation2d getRotation() {
		return rotation;
	}

	/**
	 * @param angle RADIANS -PI to PI
	 */
	public void setRotation(double angle) {
		rotation = new Rotation2d(angle);

	}

	public double getRate() {
		return rate;
	}

	/**
	 * @param rate RADIANS per second
	 */
	public void setRate(double rate) {
		this.rate = rate;
	}

	public void resetData() {
		rate = 0;
		rotation = new Rotation2d(0);
	}

}

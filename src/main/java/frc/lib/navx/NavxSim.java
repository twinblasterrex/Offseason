package frc.lib.navx;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavxSim implements Navx {

	private Rotation2d rot;
	private double rate = 0;

	public NavxSim()
	{
		rot = new Rotation2d();
	}

	@Override
	public void update(double delta)
	{
		rot = rot.plus(Rotation2d.fromRadians(getRate() * delta));
	}
	@Override
	public Rotation2d getRotation2d() {
		return rot;
	}

	@Override
	public void zeroYaw() {
		rot = new Rotation2d(0);
	}

	/**
	 *
	 * @param offset in radians
	 */
	@Override
	public void setOffset(double offset) {
		rot.plus(new Rotation2d(offset));
	}

	public double getRate() {
		return rate;
	}

	/**
	 * @param mRate In radians rotations per second
	 */
	public void setRate(double mRate) {
		rate = mRate;
	}
}

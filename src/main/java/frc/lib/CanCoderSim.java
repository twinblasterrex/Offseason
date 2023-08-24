package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class CanCoderSim {
	private double rate;
	private Rotation2d rot;
	private final int id;
	private double offset;

	public CanCoderSim(int id, double offset)
	{
		this.id = id;
		this.offset = offset;

		rot = new Rotation2d();
		rate = 0;
	}

	public void update(double delta)
	{

		rot = rot.plus(Rotation2d.fromRadians(delta * rate));
	}

	public double getRate() {
		return rate;
	}

	public void setRate(double mRate) {
		rate = mRate;
	}

	public Rotation2d getRot() {
		return rot;
	}

	public void setRot(Rotation2d mRot) {
		rot = mRot;
	}

	public double getOffset() {
		return offset;
	}

	public void setOffset(int mOffset) {
		offset = mOffset;
	}
}

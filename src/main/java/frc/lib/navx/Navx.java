package frc.lib.navx;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Navx {
	public Rotation2d getRotation2d();
	public void zeroYaw();
	public void setOffset(double offset);

	public default void setRate(double rate) {};
	public default void update(double delta) {};
}

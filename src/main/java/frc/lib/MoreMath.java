package frc.lib;

public class MoreMath {
	public static double minMax(double number, double min, double max) {
		return Math.max(Math.min(number, max), min);
	}

	public static double toMeters(double feet) // Feet to meters
	{
		return feet * 0.3048;
	}

	public static double toFeet(double meters) // Meters to feet
	{
		return meters * 3.2808399;
	}

}

package frc.robot.subsystems.Drivetrain;

public class ModuleDetails {
	public int driveID;
	public int steerID;
	public int encoderID;
	public double encoderOffset;
	public ModuleEnum module;

	public ModuleDetails(int mDriveID, int mSteerID, int mEncoderID, double mEncoderOffset, ModuleEnum mModule) {
		driveID = mDriveID;
		steerID = mSteerID;
		encoderID = mEncoderID;
		encoderOffset = mEncoderOffset;
		module = mModule;
	}
}

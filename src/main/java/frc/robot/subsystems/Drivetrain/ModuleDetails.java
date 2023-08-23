package frc.robot.subsystems.Drivetrain;

public class ModuleDetails {
	int driveID;
	int steerID;
	int encoderID;
	double encoderOffset;

	public ModuleDetails(int mDriveID, int mSteerID, int mEncoderID, double mEncoderOffset) {
		driveID = mDriveID;
		steerID = mSteerID;
		encoderID = mEncoderID;
		encoderOffset = mEncoderOffset;
	}
}

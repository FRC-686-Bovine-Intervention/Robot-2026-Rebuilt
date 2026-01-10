package frc.robot.subsystems.drive.odometry;

import edu.wpi.first.wpilibj.Timer;

public class OdometryTimestampIOSim implements OdometryTimestampIO {
	public OdometryTimestampIOSim() {}

	@Override
	public void updateInputs(OdometryTimestampIOInputs inputs) {
		inputs.timestamps = new double[] {Timer.getTimestamp()};
	}
}

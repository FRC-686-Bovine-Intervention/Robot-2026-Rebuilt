package frc.robot.subsystems.drive.odometry;

import frc.robot.subsystems.drive.odometry.OdometryThread.DoubleBuffer;

public class OdometryTimestampIOOdometryThread implements OdometryTimestampIO {
	private final DoubleBuffer timestampBuffer;

	public OdometryTimestampIOOdometryThread() {
		this.timestampBuffer = OdometryThread.getInstance().generateTimestampBuffer();
	}

	@Override
	public void updateInputs(OdometryTimestampIOInputs inputs) {
		inputs.timestamps = this.timestampBuffer.popAll();
	}
}

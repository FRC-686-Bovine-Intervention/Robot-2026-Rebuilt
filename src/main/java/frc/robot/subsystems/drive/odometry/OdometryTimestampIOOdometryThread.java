package frc.robot.subsystems.drive.odometry;

import frc.robot.subsystems.drive.odometry.OdometryThread.DoubleBuffer;

public class OdometryTimestampIOOdometryThread implements OdometryTimestampIO {
	private final DoubleBuffer timestampBuffer;

	public OdometryTimestampIOOdometryThread() {
		this.timestampBuffer = OdometryThread.getInstance().generateTimestampBuffer();
	}

	@Override
	public void updateInputs(OdometryTimestampIOInputs inputs) {
		inputs.odometrySampleCount = this.timestampBuffer.getSize();
		System.arraycopy(this.timestampBuffer.getInternalBuffer(), 0, inputs.timestamps, 0, inputs.odometrySampleCount);
		this.timestampBuffer.clear();
	}
}

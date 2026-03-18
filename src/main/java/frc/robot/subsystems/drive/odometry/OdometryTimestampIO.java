package frc.robot.subsystems.drive.odometry;

import org.littletonrobotics.junction.AutoLog;

public interface OdometryTimestampIO {
	@AutoLog
	public static class OdometryTimestampIOInputs {
		public int odometrySampleCount = 0;
		public double[] timestamps = new double[OdometryThread.MAX_BUFFER_SIZE];
	}

	public default void updateInputs(OdometryTimestampIOInputs inputs) {}
}

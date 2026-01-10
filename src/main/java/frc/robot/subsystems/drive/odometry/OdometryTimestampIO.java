package frc.robot.subsystems.drive.odometry;

import org.littletonrobotics.junction.AutoLog;

public interface OdometryTimestampIO {
	@AutoLog
	public static class OdometryTimestampIOInputs {
		public double[] timestamps = new double[0];
	}

	public default void updateInputs(OdometryTimestampIOInputs inputs) {}
}

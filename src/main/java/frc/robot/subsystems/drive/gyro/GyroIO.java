package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.drive.odometry.OdometryThread;

public interface GyroIO {
	@AutoLog
	public static class GyroIOInputs {
		public boolean connected = false;

		public int odometrySampleCount = 0;
		public Rotation3d[] odometryGyroRotation = new Rotation3d[OdometryThread.MAX_BUFFER_SIZE];

		public double yawVelocityRadsPerSec = 0.0;
		public double pitchVelocityRadsPerSec = 0.0;
		public double rollVelocityRadsPerSec = 0.0;
	}

	public default void updateInputs(GyroIOInputs inputs) {}

	public default void resetYawRads(double yawRads) {}
}

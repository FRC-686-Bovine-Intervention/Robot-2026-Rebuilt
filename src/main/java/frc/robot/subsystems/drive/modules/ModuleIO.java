package frc.robot.subsystems.drive.modules;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		boolean driveMotorConnected = false;
		boolean azimuthMotorConnected = false;
		boolean azimuthEncoderConnected = false;
		LoggedEncodedMotor driveMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor azimuthMotor = new LoggedEncodedMotor();
		LoggedEncoder azimuthEncoder = new LoggedEncoder();

		int odometrySampleCount = 0;
		double[] odometryDriveRads = new double[OdometryThread.MAX_BUFFER_SIZE];
		double[] odometryAzimuthRads = new double[OdometryThread.MAX_BUFFER_SIZE];
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ModuleIOInputs inputs) {}

	/** Run the drive motor at the specified voltage. */
	public default void setDriveVolts(double volts) {}
	public default void setDriveVelocityRadPerSec(double velocityRadPerSec, double accelerationRadPerSecSqr, double feedforwardVolts, boolean overrideWithBrakeMode) {}

	/** Run the turn motor at the specified voltage. */
	public default void setAzimuthVolts(double volts) {}
	public default void setAzimuthAngleRads(double angleRads, double feedforwardVolts) {}

	public default void stopDrive(Optional<NeutralMode> neutralMode) {}
	public default void stopAzimuth(Optional<NeutralMode> neutralMode) {}

	public default void configDrivePID(PIDGains pidGains) {}
	public default void configAzimuthPID(PIDGains pidGains) {}

	public default void configSendDrive() {}
	public default void configSendAzimuth() {}
}

package frc.robot.subsystems.drive.modules;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		boolean driveMotorConnected = false;
		boolean azimuthMotorConnected = false;
		boolean azimuthEncoderConnected = false;
		LoggedEncodedMotor driveMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor azimuthMotor = new LoggedEncodedMotor();
		LoggedEncoder azimuthEncoder = new LoggedEncoder();
		LoggedFaults driveMotorFaults = new LoggedFaults();
		LoggedFaults azimuthMotorFaults = new LoggedFaults();

		double[] odometryDriveRads = new double[0];
		double[] odometryAzimuthRads = new double[0];
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ModuleIOInputs inputs) {}

	/** Run the drive motor at the specified voltage. */
	public default void setDriveVolts(double volts) {}
	public default void setDriveVelocityRadPerSec(double velocityRadPerSec, double accelerationRadPerSec2, double feedforwardVolts, boolean overrideWithBrakeMode) {}

	/** Run the turn motor at the specified voltage. */
	public default void setAzimuthVolts(double volts) {}
	public default void setAzimuthAngleRads(double angleRads) {}

	public default void stopDrive(Optional<NeutralMode> neutralMode) {}
	public default void stopAzimuth(Optional<NeutralMode> neutralMode) {}

	public default void configDrivePID(PIDGains pidConstants) {}
	public default void configAzimuthPID(PIDGains pidConstants) {}

	public default void clearDriveStickyFaults(long bitmask) {}
	public default void clearAzimuthStickyFaults(long bitmask) {}
}

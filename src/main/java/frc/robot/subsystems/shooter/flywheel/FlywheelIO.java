package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface FlywheelIO {
	@AutoLog
	public static class FlywheelIOInputs {
		boolean masterMotorConnected = false;
		boolean slaveMotorConnected = false;
		LoggedEncodedMotor masterMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor slaveMotor = new LoggedEncodedMotor();
		double motorProfilePositionRads = 0.0;
		double motorProfileVelocityRadsPerSec = 0.0;
	}

	public default void updateInputs(FlywheelIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configProfile(double maxAcceleration, double maxJerk) {}
	public default void configFF(FFConstants ffConstants) {}
	public default void configPID(PIDConstants pidConstants) {}
}

package frc.robot.subsystems.shooter.flywheels;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface FlywheelsIO {
	@AutoLog
	public static class FlywheelsIOInputs {
		LoggedEncodedMotor leftDriverMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor rightDriverMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor kicker = new LoggedEncodedMotor();
	}

	public default void updateInputs(FlywheelsIOInputs inputs) {}

	public default void setDriverVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {}

	public default void setKickerVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {}

	public default void stopDriver(Optional<NeutralMode> neutralMode) {}
	public default void stopKicker(Optional<NeutralMode> neutralMode) {}

	public default void configDriverPID(PIDConstants pidConstants) {}

	public default void configKickerPID(PIDConstants pidConstants) {}
}

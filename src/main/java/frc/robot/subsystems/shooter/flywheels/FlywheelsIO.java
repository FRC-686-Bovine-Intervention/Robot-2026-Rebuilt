package frc.robot.subsystems.shooter.flywheels;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface FlywheelsIO {
	@AutoLog
	public static class FlywheelsIOInputs {
		boolean masterMotorConnected = false;
		boolean slaveMotorConnected = false;
		LoggedEncodedMotor masterMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor slaveMotor = new LoggedEncodedMotor();
	}

	public default void updateInputs(FlywheelsIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configPID(PIDConstants pidConstants) {}
}

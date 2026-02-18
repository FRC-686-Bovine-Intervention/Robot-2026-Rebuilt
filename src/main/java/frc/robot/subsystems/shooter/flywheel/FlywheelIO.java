package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

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
	}

	public default void updateInputs(FlywheelIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configPID(PIDConstants pidConstants) {}
}

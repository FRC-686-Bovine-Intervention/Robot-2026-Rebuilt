package frc.robot.subsystems.intake.slam;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface IntakeSlamIO {
	@AutoLog
	public static class IntakeSlamIOInputs {
		boolean motorConnected = false;
		boolean encoderConnected = false;
		LoggedEncoder encoder = new LoggedEncoder();
		LoggedEncodedMotor motor = new LoggedEncodedMotor();
	}

	public default void updateInputs(IntakeSlamIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setPositionRads(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configPID(PIDConstants pidConstants) {}

	public default void clearMotorStickyFaults(long bitmask) {}
}

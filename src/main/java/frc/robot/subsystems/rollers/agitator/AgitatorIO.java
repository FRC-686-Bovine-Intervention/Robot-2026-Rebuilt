package frc.robot.subsystems.rollers.agitator;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface AgitatorIO {
	@AutoLog
	public static class AgitatorIOInputs {
		boolean motorConnected = false;
		LoggedMotor motor = new LoggedMotor();
	}

	public default void updateInputs(AgitatorIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}
}

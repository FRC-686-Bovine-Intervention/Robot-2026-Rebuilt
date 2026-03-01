package frc.robot.subsystems.rollers.indexer;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface IndexerIO {
	@AutoLog
	public static class IndexerIOInputs {
		boolean motorConnected = false;
		LoggedMotor motor = new LoggedMotor();
	}

	public default void updateInputs(IndexerIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}
}

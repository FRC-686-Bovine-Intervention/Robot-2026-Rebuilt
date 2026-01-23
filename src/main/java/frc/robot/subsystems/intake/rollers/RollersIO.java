package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
	@AutoLog
	public static class RollersIOInputs {

	}

	public default void updateInputs(RollersIOInputs inputs) {}
}

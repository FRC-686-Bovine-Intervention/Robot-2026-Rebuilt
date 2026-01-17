package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
	@AutoLog
	public static class HoodIOInputs {

	}

	public default void updateInputs(HoodIOInputs inputs) {}
}

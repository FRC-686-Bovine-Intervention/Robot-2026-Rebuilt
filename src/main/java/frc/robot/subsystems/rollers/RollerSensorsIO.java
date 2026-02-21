package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
	@AutoLog
	public static class RollerSensorsIOInputs {
		boolean connected = false;
		boolean feederSensor = false;
	}

	public default void updateInputs(RollerSensorsIOInputs inputs) {}
}

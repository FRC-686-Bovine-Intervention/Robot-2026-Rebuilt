package frc.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
	@AutoLog
	public static class FlywheelsIOInputs {

	}

	public default void updateInputs(FlywheelsIOInputs inputs) {}
}

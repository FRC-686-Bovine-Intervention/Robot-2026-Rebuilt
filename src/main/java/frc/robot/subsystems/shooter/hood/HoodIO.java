package frc.robot.subsystems.shooter.hood;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface HoodIO {
	@AutoLog
	public static class HoodIOInputs {
		boolean motorConnected = false;
		LoggedEncodedMotor motor = new LoggedEncodedMotor();
		double motorProfilePositionRads = 0.0;
		double motorProfileVelocityRadsPerSec = 0.0;

		boolean limitSwitch = false;
	}

	public default void updateInputs(HoodIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setPositionRads(double positionRads) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configProfile(double kVVoltSecsPerRad, double kAVoltSecsSqrPerRad, double maxVelocityRadsPerSec) {}
	public default void configFF(FFGains ffGains) {}
	public default void configPID(PIDGains pidGains) {}
	public default void configSend() {}
}

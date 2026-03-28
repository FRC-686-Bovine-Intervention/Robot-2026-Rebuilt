package frc.robot.subsystems.intake.slam;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface IntakeSlamIO {
	@AutoLog
	public static class IntakeSlamIOInputs {
		boolean motorConnected = false;
		boolean encoderConnected = false;
		LoggedEncoder encoder = new LoggedEncoder();
		LoggedEncodedMotor motor = new LoggedEncodedMotor();
		double motorProfilePositionRads = 0.0;
		double motorProfileVelocityRadsPerSec = 0.0;
	}

	public default void updateInputs(IntakeSlamIOInputs inputs) {}

	public default void setVolts(double volts) {}
	
	public default void setPositionRads(double positionRads) {}
	public default void setProfile(double kVVoltSecsPerRad, double kAVoltSecsSqrPerRad, double maxVelocityRadsPerSec) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configFF(FFGains ffGains) {}
	public default void configPID(PIDGains pidGains) {}
	public default void configSend() {}
}

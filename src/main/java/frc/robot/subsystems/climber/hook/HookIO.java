package frc.robot.subsystems.climber.hook;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface HookIO {
	@AutoLog
	public static class HookIOInputs {
		boolean motorConnected = false;
		LoggedEncodedMotor motor = new LoggedEncodedMotor();
		double motorProfilePositionRads = 0.0;
		double motorProfileVelocityRadsPerSec = 0.0;

		boolean limitSwitch = false;
	}

	public default void updateInputs(HookIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setUnloadedPositionRads(double positionRads) {}
	public default void setClimbingPositionRads(double positionRads) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void setUnloadedProfile(double kVVoltSecsPerRad, double kAVoltSecsSqrPerRad, double maxVelocityRadsPerSec) {}
	public default void setClimbingProfile(double kVVoltSecsPerRad, double kAVoltSecsSqrPerRad, double maxVelocityRadsPerSec) {}
	public default void configFF(FFConstants ffConstants) {}
	public default void configPID(PIDConstants pidConstants) {}
	public default void configSend() {}
}

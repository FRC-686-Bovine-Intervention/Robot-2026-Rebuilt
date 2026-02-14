package frc.robot.subsystems.shooter.hood;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface HoodIO {
	@AutoLog
	public static class HoodIOInputs {
		boolean limitSwitch = false;
		boolean motorConnected = false;
		LoggedEncodedMotor motor = new LoggedEncodedMotor();
	}

	public default void updateInputs(HoodIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void setPositionRads(double positionRads) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configPID(PIDConstants pidConstants) {}

	public default void configFF(FFConstants ffConstants) {}
	
	public default void configProfile(double kV, double kA, double maxVelocity) {}

	public default void resetMotorPositionRads(double positionRads) {}
}

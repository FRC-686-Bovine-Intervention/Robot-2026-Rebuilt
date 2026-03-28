package frc.robot.subsystems.intake.rollers;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface IntakeRollersIO {
	@AutoLog
	public static class IntakeRollersIOInputs {
		boolean leftMotorConnected = false;
		boolean rightMotorConnected = false;
		LoggedMotor leftMotor = new LoggedMotor();
		LoggedMotor rightMotor = new LoggedMotor();
	}

	public default void updateInputs(IntakeRollersIOInputs inputs) {}

	public default void setVolts(double volts) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}
}

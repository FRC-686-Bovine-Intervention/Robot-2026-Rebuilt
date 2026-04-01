package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface FlywheelIO {
	@AutoLog
	public static class FlywheelIOInputs {
		boolean leftBottomMotorConnected = false;
		boolean leftTopMotorConnected = false;
		boolean rightBottomMotorConnected = false;
		boolean rightTopMotorConnected = false;
		LoggedEncodedMotor leftBottomMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor leftTopMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor rightBottomMotor = new LoggedEncodedMotor();
		LoggedEncodedMotor rightTopMotor = new LoggedEncodedMotor();
		double profilePositionRads = 0.0;
		double profileVelocityRadsPerSec = 0.0;
	}

	public default void updateInputs(FlywheelIOInputs inputs) {}

	public default void setVolts(double volts) {}
	public default void setCurrent(double amps) {}

	public default void setVelocityRadsPerSec(double velocityRadsPerSec) {}

	public default void stop(Optional<NeutralMode> neutralMode) {}

	public default void configProfile(double maxAccelRadsPerSecSec, double maxJerkRadsPerSecSecSec) {}
	public default void configFF(FFGains ffGains) {}
	public default void configPID(PIDGains pidGains) {}
	public default void configSend() {}
}

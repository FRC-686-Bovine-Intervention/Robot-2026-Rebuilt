package frc.robot.subsystems.shooter.hood;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.ExponentialProfile;
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

    public default void setPositionRads(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}
    
    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configTunables(PIDConstants pidConstants, double maxVelocity, double kA, double kV, double kG, double cruiseVelocity) {}

    public default void resetMotorPositionRads(double positionRads) {}
}

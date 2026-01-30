package frc.robot.subsystems.shooter.flywheels;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.hardwareID.can.CANDevice;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelsIOTalonFX implements FlywheelsIO {
	private final TalonFX leftDriverMotor;
	private final TalonFX rightDriverMotor;

	private final BaseStatusSignal[] refreshSignals;
	private final EncodedMotorStatusSignalCache leftDriverCache;
	private final EncodedMotorStatusSignalCache righttDriverCache;

	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelsIOTalonFX(CANDevice leftMotor, CANDevice rightMotor) {
		this.leftDriverMotor = leftMotor.talonFX();
		this.rightDriverMotor = rightMotor.talonFX();
		var driverConfig = new TalonFXConfiguration();
		driverConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.leftDriverMotor.getConfigurator().apply(driverConfig);
		driverConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.rightDriverMotor.getConfigurator().apply(driverConfig);
		driverConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
		;

		this.leftDriverCache = EncodedMotorStatusSignalCache.from(this.leftDriverMotor);
		this.righttDriverCache = EncodedMotorStatusSignalCache.from(this.rightDriverMotor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.leftDriverCache.encoder().position(),
			this.leftDriverCache.encoder().velocity(),
			this.leftDriverCache.motor().appliedVoltage(),
			this.leftDriverCache.motor().statorCurrent(),
			this.leftDriverCache.motor().supplyCurrent(),
			this.leftDriverCache.motor().torqueCurrent(),
			this.leftDriverCache.motor().deviceTemperature(),
			this.righttDriverCache.encoder().position(),
			this.righttDriverCache.encoder().velocity(),
			this.righttDriverCache.motor().appliedVoltage(),
			this.righttDriverCache.motor().statorCurrent(),
			this.righttDriverCache.motor().supplyCurrent(),
			this.righttDriverCache.motor().torqueCurrent(),
			this.righttDriverCache.motor().deviceTemperature(),
		};
	}

	@Override
	public void updateInputs(FlywheelsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.leftDriverMotor.updateFrom(this.leftDriverCache);
		inputs.rightDriverMotor.updateFrom(this.righttDriverCache);
	}

	@Override
	public void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.leftDriverMotor.setControl(this.flywheelVelocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
		this.rightDriverMotor.setControl(this.followerRequest.withLeaderID(this.leftDriverMotor.getDeviceID()));
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.leftDriverMotor.setControl(controlRequest);
		this.rightDriverMotor.setControl(controlRequest);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.leftDriverMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.leftDriverMotor.getConfigurator().apply(config);
		Logger.recordOutput("DEBUG/flywheel pid/p", pidConstants.kP());
		Logger.recordOutput("DEBUG/flywheel pid/i", pidConstants.kI());
		Logger.recordOutput("DEBUG/flywheel pid/d", pidConstants.kD());
	}
}

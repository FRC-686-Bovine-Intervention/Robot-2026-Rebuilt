package frc.robot.subsystems.shooter.flywheels;

import java.util.Optional;

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
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelsIOTalonFX implements FlywheelsIO {
	private final TalonFX leftDriverMotor = new TalonFX(0);
	private final TalonFX rightDriverMotor = new TalonFX(1);
	private final TalonFX kickerMotor = new TalonFX(2);

	private final BaseStatusSignal[] refreshSignals;
	private final EncodedMotorStatusSignalCache leftDriverCache;
	private final EncodedMotorStatusSignalCache righttDriverCache;
	private final EncodedMotorStatusSignalCache kickerCache;

	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
	private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelsIOTalonFX() {
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

		var kickerConfig = new TalonFXConfiguration();
		kickerConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.kickerMotor.getConfigurator().apply(kickerConfig);

		this.leftDriverCache = EncodedMotorStatusSignalCache.from(this.leftDriverMotor);
		this.righttDriverCache = EncodedMotorStatusSignalCache.from(this.rightDriverMotor);
		this.kickerCache = EncodedMotorStatusSignalCache.from(this.kickerMotor);

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
			this.kickerCache.encoder().position(),
			this.kickerCache.encoder().velocity(),
			this.kickerCache.motor().appliedVoltage(),
			this.kickerCache.motor().statorCurrent(),
			this.kickerCache.motor().supplyCurrent(),
			this.kickerCache.motor().torqueCurrent(),
			this.kickerCache.motor().deviceTemperature(),
		};
	}

	@Override
	public void updateInputs(FlywheelsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.leftDriverMotor.updateFrom(this.leftDriverCache);
		inputs.rightDriverMotor.updateFrom(this.righttDriverCache);
		inputs.kicker.updateFrom(this.kickerCache);
	}

	@Override
	public void setDriverVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.leftDriverMotor.setControl(this.flywheelVelocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
		this.rightDriverMotor.setControl(this.followerRequest.withLeaderID(this.leftDriverMotor.getDeviceID()));
	}
	@Override
	public void setKickerVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.kickerMotor.setControl(this.kickerVelocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
	}

	@Override
	public void stopDriver(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.leftDriverMotor.setControl(controlRequest);
		this.rightDriverMotor.setControl(controlRequest);
	}
	@Override
	public void stopKicker(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.kickerMotor.setControl(controlRequest);
	}

	@Override
	public void configDriverPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.leftDriverMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.leftDriverMotor.getConfigurator().apply(config);
	}
	@Override
	public void configKickerPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.kickerMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.kickerMotor.getConfigurator().apply(config);
	}
}

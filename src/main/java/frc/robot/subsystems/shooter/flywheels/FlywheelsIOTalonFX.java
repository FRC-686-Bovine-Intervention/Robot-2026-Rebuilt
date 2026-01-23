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
	private final TalonFX leftDriverMotor = new TalonFX(11);
	private final TalonFX rightDriverMotor = new TalonFX(13);
	private final TalonFX slaveleftDriverMotor = new TalonFX(12);
	private final TalonFX slaverightDriverMotor = new TalonFX(14);

	private final BaseStatusSignal[] refreshSignals;
	private final EncodedMotorStatusSignalCache leftDriverCache;
	private final EncodedMotorStatusSignalCache rightDriverCache;

	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelsIOTalonFX() {
		var driverConfig = new TalonFXConfiguration();
		driverConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.leftDriverMotor.getConfigurator().apply(driverConfig);
		this.slaveleftDriverMotor.getConfigurator().apply(driverConfig);
		driverConfig.MotorOutput
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.rightDriverMotor.getConfigurator().apply(driverConfig);
		this.slaverightDriverMotor.getConfigurator().apply(driverConfig);

		this.leftDriverCache = EncodedMotorStatusSignalCache.from(this.leftDriverMotor);
		this.rightDriverCache = EncodedMotorStatusSignalCache.from(this.rightDriverMotor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.leftDriverCache.encoder().position(),
			this.leftDriverCache.encoder().velocity(),
			this.leftDriverCache.motor().appliedVoltage(),
			this.leftDriverCache.motor().statorCurrent(),
			this.leftDriverCache.motor().supplyCurrent(),
			this.leftDriverCache.motor().torqueCurrent(),
			this.leftDriverCache.motor().deviceTemperature(),
			this.rightDriverCache.encoder().position(),
			this.rightDriverCache.encoder().velocity(),
			this.rightDriverCache.motor().appliedVoltage(),
			this.rightDriverCache.motor().statorCurrent(),
			this.rightDriverCache.motor().supplyCurrent(),
			this.rightDriverCache.motor().torqueCurrent(),
			this.rightDriverCache.motor().deviceTemperature(),
		};
	}

	@Override
	public void updateInputs(FlywheelsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.leftDriverMotor.updateFrom(this.leftDriverCache);
		inputs.rightDriverMotor.updateFrom(this.rightDriverCache);
	}

	@Override
	public void setDriverVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.leftDriverMotor.setControl(this.flywheelVelocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
		this.followerRequest.withLeaderID(this.leftDriverMotor.getDeviceID());
		this.rightDriverMotor.setControl(this.followerRequest);
		this.slaveleftDriverMotor.setControl(this.followerRequest);
		this.slaverightDriverMotor.setControl(this.followerRequest);
	}

	@Override
	public void stopDriver(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.leftDriverMotor.setControl(controlRequest);
		this.rightDriverMotor.setControl(controlRequest);
		this.slaveleftDriverMotor.setControl(controlRequest);
		this.slaverightDriverMotor.setControl(controlRequest);
	}

	@Override
	public void configDriverPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.leftDriverMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.leftDriverMotor.getConfigurator().apply(config);
	}
}

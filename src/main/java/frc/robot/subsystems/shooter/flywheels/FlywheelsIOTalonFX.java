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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.hardwareID.can.CANDevice;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelsIOTalonFX implements FlywheelsIO {
	private final TalonFX masterMotor;
	private final TalonFX slaveMotor;

	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] masterMotorConnectedSignals;
	private final BaseStatusSignal[] slaveMotorConnectedSignals;
	private final EncodedMotorStatusSignalCache masterMotorCache;
	private final EncodedMotorStatusSignalCache slaveMotorCache;

	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VoltageOut voltageRequest = new VoltageOut(0.0);
	private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelsIOTalonFX(CANDevice masterMotor, CANDevice slaveMotor) {
		this.masterMotor = masterMotor.talonFX();
		this.slaveMotor = slaveMotor.talonFX();
		var driverConfig = new TalonFXConfiguration();
		driverConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.masterMotor.getConfigurator().apply(driverConfig);
		driverConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.slaveMotor.getConfigurator().apply(driverConfig);
		driverConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
		;

		this.masterMotorCache = EncodedMotorStatusSignalCache.from(this.masterMotor);
		this.slaveMotorCache = EncodedMotorStatusSignalCache.from(this.slaveMotor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.masterMotorCache.encoder().position(),
			this.masterMotorCache.encoder().velocity(),
			this.masterMotorCache.motor().appliedVoltage(),
			this.masterMotorCache.motor().statorCurrent(),
			this.masterMotorCache.motor().supplyCurrent(),
			this.masterMotorCache.motor().torqueCurrent(),
			this.masterMotorCache.motor().deviceTemperature(),
			this.slaveMotorCache.encoder().position(),
			this.slaveMotorCache.encoder().velocity(),
			this.slaveMotorCache.motor().appliedVoltage(),
			this.slaveMotorCache.motor().statorCurrent(),
			this.slaveMotorCache.motor().supplyCurrent(),
			this.slaveMotorCache.motor().torqueCurrent(),
			this.slaveMotorCache.motor().deviceTemperature(),
		};
		this.masterMotorConnectedSignals = new BaseStatusSignal[] {
			this.masterMotorCache.encoder().position(),
			this.masterMotorCache.encoder().velocity(),
			this.masterMotorCache.motor().appliedVoltage(),
			this.masterMotorCache.motor().statorCurrent(),
			this.masterMotorCache.motor().supplyCurrent(),
			this.masterMotorCache.motor().torqueCurrent(),
			this.masterMotorCache.motor().deviceTemperature(),
		};
		this.slaveMotorConnectedSignals = new BaseStatusSignal[] {
			this.slaveMotorCache.encoder().position(),
			this.slaveMotorCache.encoder().velocity(),
			this.slaveMotorCache.motor().appliedVoltage(),
			this.slaveMotorCache.motor().statorCurrent(),
			this.slaveMotorCache.motor().supplyCurrent(),
			this.slaveMotorCache.motor().torqueCurrent(),
			this.slaveMotorCache.motor().deviceTemperature(),
		};
	}

	@Override
	public void updateInputs(FlywheelsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.masterMotorConnected = BaseStatusSignal.isAllGood(this.masterMotorConnectedSignals);
		inputs.slaveMotorConnected = BaseStatusSignal.isAllGood(this.slaveMotorConnectedSignals);
		inputs.masterMotor.updateFrom(this.masterMotorCache);
		inputs.slaveMotor.updateFrom(this.slaveMotorCache);
	}

	@Override
	public void setVolts(double volts) {
		this.masterMotor.setControl(this.voltageRequest
			.withOutput(volts)
		);
		this.slaveMotor.setControl(this.followerRequest.withLeaderID(this.masterMotor.getDeviceID()));
	}

	@Override
	public void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.masterMotor.setControl(this.velocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
		this.slaveMotor.setControl(this.followerRequest.withLeaderID(this.masterMotor.getDeviceID()));
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.masterMotor.setControl(controlRequest);
		this.slaveMotor.setControl(controlRequest);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.masterMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.masterMotor.getConfigurator().apply(config);
	}
}

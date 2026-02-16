package frc.robot.subsystems.intake.slam;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;
import frc.util.loggerUtil.inputs.LoggedEncoder.EncoderStatusSignalCache;

public class IntakeSlamIOTalonFX implements IntakeSlamIO {
	protected final TalonFX motor = HardwareDevices.intakeSlamMotorID.talonFX();
	protected final CANcoder cancoder = HardwareDevices.intakeSlamEncoderID.cancoder();

	private final EncoderStatusSignalCache encoderStatusSignalCache;
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;

	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] encoderConnectedSignals;
	private final BaseStatusSignal[] motorConnectedSignals;

	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage positionRequest = new PositionVoltage(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	public IntakeSlamIOTalonFX() {
		var encoderConfig = new CANcoderConfiguration();
		this.cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
		encoderConfig.MagnetSensor
			.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
		;
		this.cancoder.getConfigurator().apply(encoderConfig);

		var motorConfig = new TalonFXConfiguration();
		motorConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		motorConfig.Feedback
			.withRemoteCANcoder(this.cancoder)
			.withRotorToSensorRatio(IntakeSlamConstants.motorToMechanism.then(IntakeSlamConstants.sensorToMechanism.inverse()).reductionUnsigned())
			.withSensorToMechanismRatio(IntakeSlamConstants.sensorToMechanism.reductionUnsigned())
		;
		motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(true)
			// .withReverseSoftLimitThreshold(IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(IntakeSlamConstants.minAngle))
			.withReverseSoftLimitThreshold(IntakeSlamConstants.minAngle.minus(IntakeSlamConstants.cancoderZeroOffset))
			.withForwardSoftLimitEnable(true)
			// .withForwardSoftLimitThreshold(IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(IntakeSlamConstants.maxAngle))
			.withForwardSoftLimitThreshold(IntakeSlamConstants.maxAngle.minus(IntakeSlamConstants.cancoderZeroOffset))
		;
		this.motor.getConfigurator().apply(motorConfig);

		this.encoderStatusSignalCache = EncoderStatusSignalCache.from(this.cancoder);
		this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity(),
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature()
		};
		this.encoderConnectedSignals = new BaseStatusSignal[] {
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity()
		};
		this.motorConnectedSignals = new BaseStatusSignal[] {
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature()
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.encoderStatusSignalCache.getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.motor().getStatusSignals());
		this.cancoder.optimizeBusUtilization();
		this.motor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IntakeSlamIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.encoderConnected = BaseStatusSignal.isAllGood(this.encoderConnectedSignals);
		inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectedSignals);
		inputs.encoder.updateFrom(this.encoderStatusSignalCache);
		inputs.motor.updateFrom(this.motorStatusSignalCache);
	}

	@Override
	public void setVolts(double volts) {
		this.motor.setControl(this.voltageRequest
			.withOutput(volts)
		);
	}

	@Override
	public void setPositionRads(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
		this.motor.setControl(this.positionRequest
			.withPosition(Units.radiansToRotations(positionRads))
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withFeedForward(feedforwardVolts)
			.withSlot(0)
		);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.motor.setControl(controlRequest);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.motor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.motor.getConfigurator().apply(config);
	}
}

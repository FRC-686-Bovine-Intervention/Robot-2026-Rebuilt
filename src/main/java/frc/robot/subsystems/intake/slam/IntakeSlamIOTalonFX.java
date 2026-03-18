package frc.robot.subsystems.intake.slam;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;
import frc.util.loggerUtil.inputs.LoggedEncoder.EncoderStatusSignalCache;

public class IntakeSlamIOTalonFX implements IntakeSlamIO {
	// Hardware devices
	protected final TalonFX motor = HardwareDevices.intakeSlamMotorID.talonFX();
	protected final CANcoder encoder = HardwareDevices.intakeSlamEncoderID.cancoder();

	// Device configuration
	private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
	private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

	// Status Signal caches
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
	private final EncoderStatusSignalCache encoderStatusSignalCache;
	private final StatusSignal<Double> motorProfilePositionStatusSignal;
	private final StatusSignal<Double> motorProfileVelocityStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] motorConnectedSignals;
	private final BaseStatusSignal[] encoderConnectedSignals;

	// Control Requests
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	public IntakeSlamIOTalonFX() {
		// Motor Configuration
		this.motorConfig.MotorOutput
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		this.motorConfig.Feedback
			.withRemoteCANcoder(this.encoder)
			.withRotorToSensorRatio(IntakeSlamConstants.motorToMechanism.then(IntakeSlamConstants.sensorToMechanism.inverse()).reductionUnsigned())
			.withSensorToMechanismRatio(IntakeSlamConstants.sensorToMechanism.reductionUnsigned())
		;
		this.motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(true)
			.withReverseSoftLimitThreshold(IntakeSlamConstants.minAngle.minus(IntakeSlamConstants.encoderZeroOffset))
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(IntakeSlamConstants.maxAngle.minus(IntakeSlamConstants.encoderZeroOffset))
		;
		this.motorConfig.Slot0
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withGravityArmPositionOffset(IntakeSlamConstants.minAngle)
		;

		// Encoder Configuration
		this.encoder.getConfigurator().refresh(this.encoderConfig.MagnetSensor);

		this.encoderConfig.MagnetSensor
			.withSensorDirection(SensorDirectionValue.Clockwise_Positive)
		;

		// Cache Status Signals
		this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
		this.encoderStatusSignalCache = EncoderStatusSignalCache.from(this.encoder);
		this.motorProfilePositionStatusSignal = this.motor.getClosedLoopReference();
		this.motorProfileVelocityStatusSignal = this.motor.getClosedLoopReferenceSlope();

		// Make quick reference arrays
		this.refreshSignals = new BaseStatusSignal[] {
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity(),
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature(),
			this.motorProfilePositionStatusSignal,
			this.motorProfileVelocityStatusSignal,
		};
		this.motorConnectedSignals = new BaseStatusSignal[] {
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature(),
			this.motorProfilePositionStatusSignal,
			this.motorProfileVelocityStatusSignal,
		};
		this.encoderConnectedSignals = new BaseStatusSignal[] {
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity(),
		};

		// Set Status Signal update frequency
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorProfilePositionStatusSignal, this.motorProfileVelocityStatusSignal);
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2.0), this.motorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.encoderStatusSignalCache.getStatusSignals());
		this.motor.optimizeBusUtilization();
		this.encoder.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IntakeSlamIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.encoderConnected = BaseStatusSignal.isAllGood(this.encoderConnectedSignals);
		inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectedSignals);
		inputs.encoder.updateFrom(this.encoderStatusSignalCache);
		inputs.motor.updateFrom(this.motorStatusSignalCache);

		inputs.motorProfilePositionRads = Units.rotationsToRadians(this.motorProfilePositionStatusSignal.getValueAsDouble());
		inputs.motorProfileVelocityRadsPerSec = Units.rotationsToRadians(this.motorProfileVelocityStatusSignal.getValueAsDouble());
	}

	@Override
	public void setVolts(double volts) {
		this.motor.setControl(this.voltageRequest
			.withOutput(volts)
		);
	}

	@Override
	public void setPositionRads(double positionRads) {
		this.motor.setControl(this.positionRequest
			.withPosition(Units.radiansToRotations(positionRads))
		);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.motor.setControl(controlRequest);
	}

	@Override
	public void configProfile(double kVVoltSecsPerRad, double kAVoltSecsSqrPerRad, double maxVelocityRadsPerSec) {
		this.motorConfig.MotionMagic
			.withMotionMagicExpo_kV(Units.rotationsToRadians(kVVoltSecsPerRad))
			.withMotionMagicExpo_kA(Units.rotationsToRadians(kAVoltSecsSqrPerRad))
			.withMotionMagicCruiseVelocity(Units.radiansToRotations(maxVelocityRadsPerSec))
		;
	}

	@Override
	public void configFF(FFGains ffGains) {
		ffGains.update(this.motorConfig.Slot0);
	}

	@Override
	public void configPID(PIDGains pidGains) {
		pidGains.update(this.motorConfig.Slot0);
	}

	@Override
	public void configSend() {
		this.motor.getConfigurator().apply(this.motorConfig);
		this.encoder.getConfigurator().apply(this.encoderConfig);
	}
}

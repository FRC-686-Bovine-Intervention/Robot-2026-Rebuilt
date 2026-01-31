package frc.robot.subsystems.climber.hook;

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
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;
import frc.util.loggerUtil.inputs.LoggedEncoder.EncoderStatusSignalCache;

public class HookIOTalonFX implements HookIO {
	protected final TalonFX motor = HardwareDevices.climberHookMotorID.talonFX();
	protected final CANcoder cancoder = HardwareDevices.climberHookMotorID.cancoder();


	//REVIEW THIS (37-38)
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
	private final EncoderStatusSignalCache encoderStatusSignalCache;

	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage positionRequest = new PositionVoltage(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	//private double magnetOffsetRots = 0.0;

	public HookIOTalonFX() {
		var encoderConfig = new CANcoderConfiguration();
		this.cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
		//this.magnetOffsetRots = encoderConfig.MagnetSensor.MagnetOffset;
		encoderConfig.MagnetSensor
			.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
			.withAbsoluteSensorDiscontinuityPoint(0.9)
		;

		this.cancoder.getConfigurator().apply(encoderConfig);


		//REVIEW LINE 63
		var motorConfig = new TalonFXConfiguration();
		motorConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;

		motorConfig.Feedback
			.withRemoteCANcoder(this.cancoder)
			.withRotorToSensorRatio(HookConstants.motorToMechanism.then(HookConstants.sensorToMechanism.inverse()).reductionUnsigned())
			.withSensorToMechanismRatio(HookConstants.sensorToMechanism.reductionUnsigned())

			;

		motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(true)
			.withReverseSoftLimitThreshold(HookConstants.stage1LinearRelation.distanceToAngle(HookConstants.minLengthPhysical.div(HookConstants.movingStageCount)))
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(HookConstants.stage1LinearRelation.distanceToAngle(HookConstants.maxLengthSoftware.div(HookConstants.movingStageCount)));

		this.motor.getConfigurator().apply(motorConfig);

		this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
		this.encoderStatusSignalCache = EncoderStatusSignalCache.from(this.cancoder);

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.encoderStatusSignalCache.getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.motorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.cancoder));
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.cancoder));
		this.motor.optimizeBusUtilization();
		this.cancoder.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(HookIOInputs inputs) {
		BaseStatusSignal.refreshAll(
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity(),
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature()
		);
		inputs.encoderConnected = BaseStatusSignal.isAllGood(
			this.encoderStatusSignalCache.position(),
			this.encoderStatusSignalCache.velocity()
		);
		inputs.motorConnected = BaseStatusSignal.isAllGood(
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature()
		);
		inputs.encoder.updateFrom(this.encoderStatusSignalCache);
		inputs.motor.updateFrom(this.motorStatusSignalCache);
		// inputs.encoderFaults.updateFrom(this.cancoder);
		// inputs.motorFaults.updateFrom(this.motor);
		//inputs.encoderMagnetOffsetRads = Units.rotationsToRadians(this.magnetOffsetRots);
	}

	@Override
	public void setVolts(double volts) {
		this.motor.setControl(this.voltageRequest
			.withOutput(volts)
		);
	}

	@Override
	public void setPosition(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
		this.motor.setControl(this.positionRequest
			.withPosition(Units.radiansToRotations(positionRads))
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withFeedForward(feedforwardVolts)
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

	//@Override
	//public void configMagnetOffset(double positionRads) {
	// var config = new MagnetSensorConfigs();
		//this.cancoder.getConfigurator().refresh(config);
		//config.withMagnetOffset(Units.radiansToRotations(positionRads));
		//this.cancoder.getConfigurator().apply(config);
		//this.magnetOffsetRots = Units.radiansToRotations(positionRads);
	// }

	//@Override
	//public void clearMotorStickyFaults(long bitmask) {
		//if (bitmask == DeviceFaults.noneMask) {return;}
		//if (bitmask == DeviceFaults.allMask) {
			//this.motor.clearStickyFaults();
		//} else {
			//for (var faultType : FaultType.possibleTalonFXFaults) {
			// if (faultType.isPartOf(bitmask)) {
					//faultType.clearStickyFaultOn(this.motor);
				//}
		// }
	// }
	//}


	//@Override
	//public void clearEncoderStickyFaults(long bitmask) {
		//if (bitmask == DeviceFaults.noneMask) {return;}
		//if (bitmask == DeviceFaults.allMask) {
			//this.cancoder.clearStickyFaults();
		//} else {
			//for (var faultType : FaultType.possibleCancoderFaults) {
				//if (faultType.isPartOf(bitmask)) {
					//faultType.clearStickyFaultOn(this.cancoder);
				//}
			//}
		//}
	//}
}

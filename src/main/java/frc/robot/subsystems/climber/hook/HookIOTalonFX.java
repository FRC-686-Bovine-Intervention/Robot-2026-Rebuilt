package frc.robot.subsystems.climber.hook;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class HookIOTalonFX implements HookIO {
	// Hardware devices
	protected final TalonFX motor = HardwareDevices.climberHookMotorID.talonFX();

	// Status Signal caches
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
	private final StatusSignal<Double> motorProfilePositionStatusSignal;
	private final StatusSignal<Double> motorProfileVelocityStatusSignal;
	private final StatusSignal<Boolean> limitSwitchStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] motorConnectionSignals;

	// Control Requests
	private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
	private final DynamicMotionMagicExpoVoltage unloadedPositionRequest = new DynamicMotionMagicExpoVoltage(0.0, 0.0, 0.0).withEnableFOC(true);
	private final DynamicMotionMagicExpoVoltage climbingPositionRequest = new DynamicMotionMagicExpoVoltage(0.0, 0.0, 0.0).withEnableFOC(true);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	public HookIOTalonFX() {
		var motorConfig = new TalonFXConfiguration();

		motorConfig.MotorOutput
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(false)
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.distanceToAngle(HookConstants.maxLength)))
		;
		motorConfig.HardwareLimitSwitch
			.withReverseLimitEnable(true)
			.withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
			.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
			.withReverseLimitAutosetPositionEnable(true)
			.withReverseLimitAutosetPositionValue(HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.distanceToAngle(HookConstants.minLength)))
			.withForwardLimitEnable(false)
		;
		motorConfig.Slot0
			.withGravityType(GravityTypeValue.Elevator_Static)
		;

		this.motor.getConfigurator().apply(motorConfig);

		this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
		this.motorProfilePositionStatusSignal = this.motor.getClosedLoopReference();
		this.motorProfileVelocityStatusSignal = this.motor.getClosedLoopReferenceSlope();
		this.limitSwitchStatusSignal = this.motor.getFault_ReverseHardLimit();

		this.refreshSignals = new BaseStatusSignal[] {
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature(),
			this.motorProfilePositionStatusSignal,
			this.motorProfileVelocityStatusSignal,
			this.limitSwitchStatusSignal,
		};
		this.motorConnectionSignals = new BaseStatusSignal[] {
			this.motorStatusSignalCache.encoder().position(),
			this.motorStatusSignalCache.encoder().velocity(),
			this.motorStatusSignalCache.motor().appliedVoltage(),
			this.motorStatusSignalCache.motor().statorCurrent(),
			this.motorStatusSignalCache.motor().supplyCurrent(),
			this.motorStatusSignalCache.motor().torqueCurrent(),
			this.motorStatusSignalCache.motor().deviceTemperature(),
			this.motorProfilePositionStatusSignal,
			this.motorProfileVelocityStatusSignal,
			this.limitSwitchStatusSignal,
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorProfilePositionStatusSignal, this.motorProfileVelocityStatusSignal);
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.motorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.limitSwitchStatusSignal);
		this.motor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(HookIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectionSignals);
		inputs.motor.updateFrom(this.motorStatusSignalCache);
		inputs.motorProfilePositionRads = Units.rotationsToRadians(this.motorProfilePositionStatusSignal.getValueAsDouble());
		inputs.motorProfileVelocityRadsPerSec = Units.rotationsToRadians(this.motorProfileVelocityStatusSignal.getValueAsDouble());

		inputs.limitSwitch = this.limitSwitchStatusSignal.getValue();
	}

	@Override
	public void setVolts(double volts) {
		this.motor.setControl(this.voltageRequest
			.withOutput(volts)
		);
	}

	@Override
	public void setUnloadedPosition(double positionRads) {
		this.motor.setControl(this.unloadedPositionRequest
			.withPosition(Units.radiansToRotations(positionRads))
		);
	}

	@Override
	public void setClimbingPosition(double positionRads) {
		this.motor.setControl(this.climbingPositionRequest
			.withPosition(Units.radiansToRotations(positionRads))
		);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.motor.setControl(controlRequest);
	}

	@Override
	public void configUnloadedProfile(double kV, double kA, double maxVelocityRadsPerSec) {
		this.unloadedPositionRequest
			.withKV(kV)
			.withKA(kA)
			.withVelocity(Units.radiansToRotations(maxVelocityRadsPerSec))
		;
	}

	@Override
	public void configClimbingProfile(double kV, double kA, double maxVelocityRadsPerSec) {
		this.climbingPositionRequest
			.withKV(kV)
			.withKA(kA)
			.withVelocity(Units.radiansToRotations(maxVelocityRadsPerSec))
		;
	}

	@Override
	public void configFF(FFConstants ffConstants) {
		var config = new Slot0Configs();
		this.motor.getConfigurator().refresh(config);
		ffConstants.update(config);
		this.motor.getConfigurator().apply(config);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.motor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.motor.getConfigurator().apply(config);
	}
}

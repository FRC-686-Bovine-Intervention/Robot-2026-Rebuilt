package frc.robot.subsystems.shooter.hood;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class HoodIOTalonFXS implements HoodIO {
	// Hardware devices
	protected final TalonFXS motor = HardwareDevices.hoodMotorID.talonFXS();

	// Status Signal caches
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
	private final StatusSignal<Double> motorProfilePositionStatusSignal;
	private final StatusSignal<Double> motorProfileVelocityStatusSignal;
	private final StatusSignal<ReverseLimitValue> limitSwitchStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] motorConnectionSignals;

	// Control Requests
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();
	private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
	private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0.0).withEnableFOC(false);

	public HoodIOTalonFXS() {
		var motorConfig = new TalonFXSConfiguration();

		motorConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(false)
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(HoodConstants.motorToMechanism.inverse().applyUnsigned(HoodConstants.maxAngle))
		;
		// motorConfig.HardwareLimitSwitch
		// 	.withReverseLimitEnable(true)
		// 	.withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS1)
		// 	.withReverseLimitRemoteSensorID(HardwareDevices.candiID.id)
		// 	.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
		// 	.withForwardLimitEnable(false)
		// ;
		motorConfig.Slot0
			.withGravityType(GravityTypeValue.Arm_Cosine)
		;

		this.motor.getConfigurator().apply(motorConfig);

		this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
		this.motorProfilePositionStatusSignal = this.motor.getClosedLoopReference();
		this.motorProfileVelocityStatusSignal = this.motor.getClosedLoopReferenceSlope();
		this.limitSwitchStatusSignal = this.motor.getReverseLimit();

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
	public void updateInputs(HoodIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectionSignals);
		inputs.motor.updateFrom(this.motorStatusSignalCache);
		inputs.motorProfilePositionRads = Units.rotationsToRadians(this.motorProfilePositionStatusSignal.getValueAsDouble());
		inputs.motorProfileVelocityRadsPerSec = Units.rotationsToRadians(this.motorProfileVelocityStatusSignal.getValueAsDouble());

		inputs.limitSwitch = this.limitSwitchStatusSignal.getValue() == ReverseLimitValue.ClosedToGround;
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
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.motor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.motor.getConfigurator().apply(config);
	}

	@Override
	public void configFF(FFConstants ffConstants) {
		var config = new Slot0Configs();
		this.motor.getConfigurator().refresh(config);
		ffConstants.update(config);
		this.motor.getConfigurator().apply(config);
	}

	@Override
	public void configProfile(double kV, double kA, double maxVelocity) {
		var config = new MotionMagicConfigs();
		this.motor.getConfigurator().refresh(config);
		config
			.withMotionMagicExpo_kV(kV)
			.withMotionMagicExpo_kA(kA)
			.withMotionMagicCruiseVelocity(maxVelocity)
		;
		this.motor.getConfigurator().apply(config);
	}

	@Override
	public void resetMotorPositionRads(double positionRads) {
		this.motor.setPosition(Units.radiansToRotations(positionRads));
	}
}

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.commonDevices.CommonCANdi;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class HoodIOTalonFXS implements HoodIO {
	// Hardware devices
	protected final TalonFXS motor = HardwareDevices.hoodMotorID.talonFXS();

	// Device configuration
	private final TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();

	// Status Signal caches
	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
	private final StatusSignal<Double> motorProfilePositionStatusSignal;
	private final StatusSignal<Double> motorProfileVelocityStatusSignal;
	private final StatusSignal<Boolean> limitSwitchStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] motorConnectionSignals;

	// Control Requests
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();
	private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
	private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0.0).withEnableFOC(false);

	public HoodIOTalonFXS(CommonCANdi candi) {
		this.motorConfig.MotorOutput
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		this.motorConfig.Commutation
			.withMotorArrangement(MotorArrangementValue.NEO550_JST)
		;
		this.motorConfig.ExternalFeedback
			.withSensorToMechanismRatio(HoodConstants.motorToMechanism.reductionUnsigned())
		;
		this.motorConfig.CurrentLimits
			.withStatorCurrentLimitEnable(true)
			.withStatorCurrentLimit(Amps.of(40.0))
		;
		this.motorConfig.SoftwareLimitSwitch
			.withReverseSoftLimitEnable(false)
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(HoodConstants.maxAngle)
		;
		this.motorConfig.HardwareLimitSwitch
			.withReverseLimitEnable(true)
			.withReverseLimitRemoteCANdiS1(candi.candi)
			.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
			.withReverseLimitAutosetPositionEnable(true)
			.withReverseLimitAutosetPositionValue(HoodConstants.limitSwitchAngle)
			.withForwardLimitEnable(false)
		;
		this.motorConfig.Slot0
			.withGravityType(GravityTypeValue.Arm_Cosine)
		;

		this.motor.getConfigurator().apply(this.motorConfig);

		candi.candiConfig.DigitalInputs
			.withS1FloatState(S1FloatStateValue.FloatDetect)
			.withS1CloseState(S1CloseStateValue.CloseWhenLow)
		;

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

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, candi.candi.getS1Closed());
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
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
			// .withMotionMagicExpo_kV(Units.rotationsToRadians(kVVoltSecsPerRad))
			// .withMotionMagicExpo_kA(Units.rotationsToRadians(kAVoltSecsSqrPerRad))
			.withMotionMagicExpo_kV(kVVoltSecsPerRad)
			.withMotionMagicExpo_kA(kAVoltSecsSqrPerRad)
			.withMotionMagicCruiseVelocity(Units.radiansToRotations(maxVelocityRadsPerSec))
		;
	}

	@Override
	public void configFF(FFConstants ffConstants) {
		ffConstants.update(this.motorConfig.Slot0);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		pidConstants.update(this.motorConfig.Slot0);
	}

	@Override
	public void configSend() {
		this.motor.getConfigurator().apply(this.motorConfig);
	}
}

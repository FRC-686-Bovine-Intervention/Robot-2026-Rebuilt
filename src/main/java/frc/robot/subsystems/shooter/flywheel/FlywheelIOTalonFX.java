package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelIOTalonFX implements FlywheelIO {
	// Hardware devices
	protected final TalonFX leftBottomMotor = HardwareDevices.leftBottomFlywheelMotorID.talonFX();
	protected final TalonFX leftTopMotor = HardwareDevices.leftTopFlywheelMotorID.talonFX();
	protected final TalonFX rightBottomMotor = HardwareDevices.rightBottomFlywheelMotorID.talonFX();
	protected final TalonFX rightTopMotor = HardwareDevices.rightTopFlywheelMotorID.talonFX();

	// Device configuration
	private final TalonFXConfiguration leftBottomConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration leftTopConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration rightBottomConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration rightTopConfig = new TalonFXConfiguration();

	// Status Signal caches
	private final EncodedMotorStatusSignalCache leftBottomMotorStatusSignalCache;
	private final EncodedMotorStatusSignalCache leftTopMotorStatusSignalCache;
	private final EncodedMotorStatusSignalCache rightBottomMotorStatusSignalCache;
	private final EncodedMotorStatusSignalCache rightTopMotorStatusSignalCache;
	private final StatusSignal<Double> profilePositionStatusSignal;
	private final StatusSignal<Double> profileVelocityStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] leftBottomMotorConnectedSignals;
	private final BaseStatusSignal[] leftTopMotorConnectedSignals;
	private final BaseStatusSignal[] rightBottomMotorConnectedSignals;
	private final BaseStatusSignal[] rightTopMotorConnectedSignals;

	// Control Requests
	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VoltageOut voltageRequest = new VoltageOut(0.0);
	private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelIOTalonFX() {
		// Motor Configuration
		this.leftBottomConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.leftBottomConfig.Feedback
			.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
		;
		this.leftBottomConfig.Voltage
			.withPeakReverseVoltage(0.0)
		;

		this.leftTopConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.leftTopConfig.Feedback
			.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
		;
		this.leftTopConfig.Voltage
			.withPeakReverseVoltage(0.0)
		;

		this.rightBottomConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.rightBottomConfig.Feedback
			.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
		;
		this.rightBottomConfig.Voltage
			.withPeakReverseVoltage(0.0)
		;

		this.rightTopConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.rightTopConfig.Feedback
			.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
		;
		this.rightTopConfig.Voltage
			.withPeakReverseVoltage(0.0)
		;

		// Cache Status Signals
		this.leftBottomMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.leftBottomMotor);
		this.leftTopMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.leftTopMotor);
		this.rightBottomMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.rightBottomMotor);
		this.rightTopMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.rightTopMotor);
		this.profilePositionStatusSignal = this.leftBottomMotor.getClosedLoopReference();
		this.profileVelocityStatusSignal = this.leftBottomMotor.getClosedLoopReferenceSlope();

		// Make quick reference arrays
		this.refreshSignals = new BaseStatusSignal[] {
			this.leftBottomMotorStatusSignalCache.encoder().position(),
			this.leftBottomMotorStatusSignalCache.encoder().velocity(),
			this.leftBottomMotorStatusSignalCache.motor().appliedVoltage(),
			this.leftBottomMotorStatusSignalCache.motor().statorCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().supplyCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().torqueCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().deviceTemperature(),
			this.profilePositionStatusSignal,
			this.profileVelocityStatusSignal,
			this.leftTopMotorStatusSignalCache.encoder().position(),
			this.leftTopMotorStatusSignalCache.encoder().velocity(),
			this.leftTopMotorStatusSignalCache.motor().appliedVoltage(),
			this.leftTopMotorStatusSignalCache.motor().statorCurrent(),
			this.leftTopMotorStatusSignalCache.motor().supplyCurrent(),
			this.leftTopMotorStatusSignalCache.motor().torqueCurrent(),
			this.leftTopMotorStatusSignalCache.motor().deviceTemperature(),
			this.rightBottomMotorStatusSignalCache.encoder().position(),
			this.rightBottomMotorStatusSignalCache.encoder().velocity(),
			this.rightBottomMotorStatusSignalCache.motor().appliedVoltage(),
			this.rightBottomMotorStatusSignalCache.motor().statorCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().supplyCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().torqueCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().deviceTemperature(),
			this.rightTopMotorStatusSignalCache.encoder().position(),
			this.rightTopMotorStatusSignalCache.encoder().velocity(),
			this.rightTopMotorStatusSignalCache.motor().appliedVoltage(),
			this.rightTopMotorStatusSignalCache.motor().statorCurrent(),
			this.rightTopMotorStatusSignalCache.motor().supplyCurrent(),
			this.rightTopMotorStatusSignalCache.motor().torqueCurrent(),
			this.rightTopMotorStatusSignalCache.motor().deviceTemperature(),
		};

		this.leftBottomMotorConnectedSignals = new BaseStatusSignal[] {
			this.leftBottomMotorStatusSignalCache.encoder().position(),
			this.leftBottomMotorStatusSignalCache.encoder().velocity(),
			this.leftBottomMotorStatusSignalCache.motor().appliedVoltage(),
			this.leftBottomMotorStatusSignalCache.motor().statorCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().supplyCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().torqueCurrent(),
			this.leftBottomMotorStatusSignalCache.motor().deviceTemperature(),
			this.profilePositionStatusSignal,
			this.profileVelocityStatusSignal,
		};

		this.leftTopMotorConnectedSignals = new BaseStatusSignal[] {
			this.leftTopMotorStatusSignalCache.encoder().position(),
			this.leftTopMotorStatusSignalCache.encoder().velocity(),
			this.leftTopMotorStatusSignalCache.motor().appliedVoltage(),
			this.leftTopMotorStatusSignalCache.motor().statorCurrent(),
			this.leftTopMotorStatusSignalCache.motor().supplyCurrent(),
			this.leftTopMotorStatusSignalCache.motor().torqueCurrent(),
			this.leftTopMotorStatusSignalCache.motor().deviceTemperature(),
		};

		this.rightBottomMotorConnectedSignals = new BaseStatusSignal[] {
			this.rightBottomMotorStatusSignalCache.encoder().position(),
			this.rightBottomMotorStatusSignalCache.encoder().velocity(),
			this.rightBottomMotorStatusSignalCache.motor().appliedVoltage(),
			this.rightBottomMotorStatusSignalCache.motor().statorCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().supplyCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().torqueCurrent(),
			this.rightBottomMotorStatusSignalCache.motor().deviceTemperature(),
		};

		this.rightTopMotorConnectedSignals = new BaseStatusSignal[] {
			this.rightTopMotorStatusSignalCache.encoder().position(),
			this.rightTopMotorStatusSignalCache.encoder().velocity(),
			this.rightTopMotorStatusSignalCache.motor().appliedVoltage(),
			this.rightTopMotorStatusSignalCache.motor().statorCurrent(),
			this.rightTopMotorStatusSignalCache.motor().supplyCurrent(),
			this.rightTopMotorStatusSignalCache.motor().torqueCurrent(),
			this.rightTopMotorStatusSignalCache.motor().deviceTemperature(),
		};

		// Set Status Signal update frequency
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.leftBottomMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.leftBottomMotorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.profilePositionStatusSignal, this.profileVelocityStatusSignal);
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.leftTopMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.leftTopMotorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.rightBottomMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.rightBottomMotorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.rightTopMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.rightTopMotorStatusSignalCache.motor().getStatusSignals());
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.leftBottomMotorConnected = BaseStatusSignal.isAllGood(this.leftBottomMotorConnectedSignals);
		inputs.leftTopMotorConnected = BaseStatusSignal.isAllGood(this.leftTopMotorConnectedSignals);
		inputs.rightBottomMotorConnected = BaseStatusSignal.isAllGood(this.rightBottomMotorConnectedSignals);
		inputs.rightTopMotorConnected = BaseStatusSignal.isAllGood(this.rightTopMotorConnectedSignals);

		inputs.leftBottomMotor.updateFrom(this.leftBottomMotorStatusSignalCache);
		inputs.leftTopMotor.updateFrom(this.leftTopMotorStatusSignalCache);
		inputs.rightBottomMotor.updateFrom(this.rightBottomMotorStatusSignalCache);
		inputs.rightTopMotor.updateFrom(this.rightTopMotorStatusSignalCache);

		inputs.profilePositionRads = Units.rotationsToRadians(this.profilePositionStatusSignal.getValueAsDouble());
		inputs.profileVelocityRadsPerSec = Units.rotationsToRadians(this.profileVelocityStatusSignal.getValueAsDouble());
	}

	@Override
	public void setVolts(double volts) {
		this.leftBottomMotor.setControl(this.voltageRequest
			.withOutput(volts)
		);
		this.followerRequest.withLeaderID(this.leftBottomMotor.getDeviceID());
		this.leftTopMotor.setControl(this.followerRequest);
		this.rightBottomMotor.setControl(this.followerRequest);
		this.rightTopMotor.setControl(this.followerRequest);
	}

	@Override
	public void setVelocityRadsPerSec(double velocityRadsPerSec) {
		this.leftBottomMotor.setControl(this.velocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
		);
		this.followerRequest.withLeaderID(this.leftBottomMotor.getDeviceID());
		this.leftTopMotor.setControl(this.followerRequest);
		this.rightBottomMotor.setControl(this.followerRequest);
		this.rightTopMotor.setControl(this.followerRequest);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.leftBottomMotor.setControl(controlRequest);
		this.leftTopMotor.setControl(controlRequest);
		this.rightBottomMotor.setControl(controlRequest);
		this.rightTopMotor.setControl(controlRequest);
	}

	@Override
	public void configProfile(double maxAccelRadsPerSecSec, double maxJerkRadsPerSecSecSec) {
		this.leftBottomConfig.MotionMagic
			.withMotionMagicAcceleration(Units.radiansToRotations(maxAccelRadsPerSecSec))
			.withMotionMagicJerk(Units.radiansToRotations(maxJerkRadsPerSecSecSec))
		;
	}

	@Override
	public void configFF(FFGains ffGains) {
		ffGains.update(this.leftBottomConfig.Slot0);
	}

	@Override
	public void configPID(PIDGains pidGains) {
		pidGains.update(this.leftBottomConfig.Slot0);
	}

	@Override
	public void configSend() {
		this.leftBottomMotor.getConfigurator().apply(this.leftBottomConfig);
		this.leftTopMotor.getConfigurator().apply(this.leftTopConfig);
		this.rightBottomMotor.getConfigurator().apply(this.rightBottomConfig);
		this.rightTopMotor.getConfigurator().apply(this.rightTopConfig);
	}
}

package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelConfig;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelIOTalonFX implements FlywheelIO {
	private final FlywheelConfig config;
	// Hardware devices
	protected final TalonFX masterMotor;
	protected final TalonFX slaveMotor;

	// Status Signal caches
	private final EncodedMotorStatusSignalCache masterMotorStatusSignalCache;
	private final EncodedMotorStatusSignalCache slaveMotorStatusSignalCache;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] masterMotorConnectedSignals;
	private final BaseStatusSignal[] slaveMotorConnectedSignals;

	// Control Requests
	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VoltageOut voltageRequest = new VoltageOut(0.0);
	private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelIOTalonFX(FlywheelConfig config) {
		this.config = config;
		// Construct hardware devices
		this.masterMotor = this.config.masterMotorID.talonFX();
		this.slaveMotor = this.config.slaveMotorID.talonFX();

		// Motor Configuration
		var motorConfig = new TalonFXConfiguration();
		motorConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.masterMotor.getConfigurator().apply(motorConfig);
		motorConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
		;
		this.slaveMotor.getConfigurator().apply(motorConfig);

		// Cache Status Signals
		this.masterMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.masterMotor);
		this.slaveMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.slaveMotor);

		// Make quick reference arrays
		this.refreshSignals = new BaseStatusSignal[] {
			this.masterMotorStatusSignalCache.encoder().position(),
			this.masterMotorStatusSignalCache.encoder().velocity(),
			this.masterMotorStatusSignalCache.motor().appliedVoltage(),
			this.masterMotorStatusSignalCache.motor().statorCurrent(),
			this.masterMotorStatusSignalCache.motor().supplyCurrent(),
			this.masterMotorStatusSignalCache.motor().torqueCurrent(),
			this.masterMotorStatusSignalCache.motor().deviceTemperature(),
			this.slaveMotorStatusSignalCache.encoder().position(),
			this.slaveMotorStatusSignalCache.encoder().velocity(),
			this.slaveMotorStatusSignalCache.motor().appliedVoltage(),
			this.slaveMotorStatusSignalCache.motor().statorCurrent(),
			this.slaveMotorStatusSignalCache.motor().supplyCurrent(),
			this.slaveMotorStatusSignalCache.motor().torqueCurrent(),
			this.slaveMotorStatusSignalCache.motor().deviceTemperature(),
		};
		this.masterMotorConnectedSignals = new BaseStatusSignal[] {
			this.masterMotorStatusSignalCache.encoder().position(),
			this.masterMotorStatusSignalCache.encoder().velocity(),
			this.masterMotorStatusSignalCache.motor().appliedVoltage(),
			this.masterMotorStatusSignalCache.motor().statorCurrent(),
			this.masterMotorStatusSignalCache.motor().supplyCurrent(),
			this.masterMotorStatusSignalCache.motor().torqueCurrent(),
			this.masterMotorStatusSignalCache.motor().deviceTemperature(),
		};
		this.slaveMotorConnectedSignals = new BaseStatusSignal[] {
			this.slaveMotorStatusSignalCache.encoder().position(),
			this.slaveMotorStatusSignalCache.encoder().velocity(),
			this.slaveMotorStatusSignalCache.motor().appliedVoltage(),
			this.slaveMotorStatusSignalCache.motor().statorCurrent(),
			this.slaveMotorStatusSignalCache.motor().supplyCurrent(),
			this.slaveMotorStatusSignalCache.motor().torqueCurrent(),
			this.slaveMotorStatusSignalCache.motor().deviceTemperature(),
		};

		// Set Status Signal update frequency
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.masterMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), this.slaveMotorStatusSignalCache.motor().getStatusSignals());
		this.masterMotor.optimizeBusUtilization();
		this.slaveMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.masterMotorConnected = BaseStatusSignal.isAllGood(this.masterMotorConnectedSignals);
		inputs.slaveMotorConnected = BaseStatusSignal.isAllGood(this.slaveMotorConnectedSignals);
		inputs.masterMotor.updateFrom(this.masterMotorStatusSignalCache);
		inputs.slaveMotor.updateFrom(this.slaveMotorStatusSignalCache);
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
	public void configProfile(double maxAccelerationRadsPerSecSec, double maxJerkRadsPerSecSecSec) {
		var config = new MotionMagicConfigs();
		this.masterMotor.getConfigurator().refresh(config);
		config
			.withMotionMagicAcceleration(Units.radiansToRotations(maxAccelerationRadsPerSecSec))
			.withMotionMagicJerk(Units.radiansToRotations(maxJerkRadsPerSecSecSec))
		;
		this.masterMotor.getConfigurator().apply(config);
	}

	@Override
	public void configFF(FFConstants ffConstants) {
		var config = new Slot0Configs();
		this.masterMotor.getConfigurator().refresh(config);
		ffConstants.update(config);
		this.masterMotor.getConfigurator().apply(config);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.masterMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.masterMotor.getConfigurator().apply(config);
	}
}

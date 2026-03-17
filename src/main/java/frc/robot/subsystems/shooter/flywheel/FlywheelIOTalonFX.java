package frc.robot.subsystems.shooter.flywheel;

import java.util.ArrayList;
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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelConfig;
import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelIOTalonFX implements FlywheelIO {
	private final FlywheelConfig config;
	// Hardware devices
	protected final TalonFX masterMotor;
	protected final TalonFX[] slaveMotors = new TalonFX[3];

	// Device configuration
	private final TalonFXConfiguration masterConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration[] slaveConfigs = new TalonFXConfiguration[] {
		new TalonFXConfiguration(),
		new TalonFXConfiguration(),
		new TalonFXConfiguration()
	};

	// Status Signal caches
	private final EncodedMotorStatusSignalCache masterMotorStatusSignalCache;
	private final EncodedMotorStatusSignalCache[] slaveMotorStatusSignalCaches = new EncodedMotorStatusSignalCache[3];
	private final StatusSignal<Double> motorProfilePositionStatusSignal;
	private final StatusSignal<Double> motorProfileVelocityStatusSignal;

	// Quick reference arrays
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] masterMotorConnectedSignals;
	private final BaseStatusSignal[][] slaveMotorConnectedSignals = new BaseStatusSignal[3][];

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
		for (int i = 0; i < this.slaveMotors.length; i++) {
			this.slaveMotors[i] = this.config.slaveMotorIDs[i].talonFX();
		}

		// Motor Configuration
		this.masterConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(config.masterInvertedValue)
		;
		this.masterConfig.Feedback
			.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
		;

		for (int i = 0; i < this.slaveConfigs.length; i++) {
			var slaveConfig = this.slaveConfigs[i];
			slaveConfig.MotorOutput
				.withNeutralMode(NeutralModeValue.Coast)
				.withInverted(config.slaveInvertedValues[i])
			;

			slaveConfig.Feedback
				.withSensorToMechanismRatio(FlywheelConstants.motorToMechanism.reductionUnsigned())
			;
		}

		// Cache Status Signals
		this.masterMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.masterMotor);
		for (int i = 0; i < this.slaveMotorStatusSignalCaches.length; i++) {
			this.slaveMotorStatusSignalCaches[i] = EncodedMotorStatusSignalCache.from(this.slaveMotors[i]);
		}
		this.motorProfilePositionStatusSignal = this.masterMotor.getClosedLoopReference();
		this.motorProfileVelocityStatusSignal = this.masterMotor.getClosedLoopReferenceSlope();

		// Make quick reference arrays
		var refreshSignals = new ArrayList<BaseStatusSignal>();
		refreshSignals.add(this.masterMotorStatusSignalCache.encoder().position());
		refreshSignals.add(this.masterMotorStatusSignalCache.encoder().velocity());
		refreshSignals.add(this.masterMotorStatusSignalCache.motor().appliedVoltage());
		refreshSignals.add(this.masterMotorStatusSignalCache.motor().statorCurrent());
		refreshSignals.add(this.masterMotorStatusSignalCache.motor().supplyCurrent());
		refreshSignals.add(this.masterMotorStatusSignalCache.motor().torqueCurrent());
		refreshSignals.add(this.masterMotorStatusSignalCache.motor().deviceTemperature());

		for (int i = 0; i < this.slaveMotorStatusSignalCaches.length; i++) {
			var slaveMotorStatusSignalCache = this.slaveMotorStatusSignalCaches[i];
			refreshSignals.add(slaveMotorStatusSignalCache.encoder().position());
			refreshSignals.add(slaveMotorStatusSignalCache.encoder().velocity());
			refreshSignals.add(slaveMotorStatusSignalCache.motor().appliedVoltage());
			refreshSignals.add(slaveMotorStatusSignalCache.motor().statorCurrent());
			refreshSignals.add(slaveMotorStatusSignalCache.motor().supplyCurrent());
			refreshSignals.add(slaveMotorStatusSignalCache.motor().torqueCurrent());
			refreshSignals.add(slaveMotorStatusSignalCache.motor().deviceTemperature());
		}

		refreshSignals.add(this.motorProfilePositionStatusSignal);
		refreshSignals.add(this.motorProfileVelocityStatusSignal);

		this.refreshSignals = refreshSignals.toArray(BaseStatusSignal[]::new);

		this.masterMotorConnectedSignals = new BaseStatusSignal[] {
			this.masterMotorStatusSignalCache.encoder().position(),
			this.masterMotorStatusSignalCache.encoder().velocity(),
			this.masterMotorStatusSignalCache.motor().appliedVoltage(),
			this.masterMotorStatusSignalCache.motor().statorCurrent(),
			this.masterMotorStatusSignalCache.motor().supplyCurrent(),
			this.masterMotorStatusSignalCache.motor().torqueCurrent(),
			this.masterMotorStatusSignalCache.motor().deviceTemperature(),
			this.motorProfilePositionStatusSignal,
			this.motorProfileVelocityStatusSignal,
		};

		for (int i = 0; i < this.slaveMotorConnectedSignals.length; i++) {
			var slaveMotorStatusSignalCache = this.slaveMotorStatusSignalCaches[i];
			this.slaveMotorConnectedSignals[i] = new BaseStatusSignal[] {
				slaveMotorStatusSignalCache.encoder().position(),
				slaveMotorStatusSignalCache.encoder().velocity(),
				slaveMotorStatusSignalCache.motor().appliedVoltage(),
				slaveMotorStatusSignalCache.motor().statorCurrent(),
				slaveMotorStatusSignalCache.motor().supplyCurrent(),
				slaveMotorStatusSignalCache.motor().torqueCurrent(),
				slaveMotorStatusSignalCache.motor().deviceTemperature(),
			};
		}

		// Set Status Signal update frequency
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.masterMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.masterMotorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorProfilePositionStatusSignal, this.motorProfileVelocityStatusSignal);
		for (int i = 0; i < this.slaveMotorStatusSignalCaches.length; i++) {
			var slaveMotorStatusSignalCache = this.slaveMotorStatusSignalCaches[i];
			BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), slaveMotorStatusSignalCache.encoder().getStatusSignals());
			BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(10.0), slaveMotorStatusSignalCache.motor().getStatusSignals());
		}
		this.masterMotor.optimizeBusUtilization();
		for (var slaveMotor : this.slaveMotors) {
			slaveMotor.optimizeBusUtilization();
		}
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.masterMotorConnected = BaseStatusSignal.isAllGood(this.masterMotorConnectedSignals);
		for (int i = 0; i < this.slaveMotorConnectedSignals.length; i++) {
			inputs.slaveMotorsConnected[i] = BaseStatusSignal.isAllGood(this.slaveMotorConnectedSignals[i]);
		}
		inputs.masterMotor.updateFrom(this.masterMotorStatusSignalCache);
		for (int i = 0; i < this.slaveMotors.length; i++) {
			inputs.slaveMotors[i].updateFrom(this.slaveMotorStatusSignalCaches[i]);
		}
		inputs.motorProfilePositionRads = Units.rotationsToRadians(this.motorProfilePositionStatusSignal.getValueAsDouble());
		inputs.motorProfileVelocityRadsPerSec = Units.rotationsToRadians(this.motorProfileVelocityStatusSignal.getValueAsDouble());
	}

	@Override
	public void setVolts(double volts) {
		this.masterMotor.setControl(this.voltageRequest
			.withOutput(volts)
		);
		for (int i = 0; i < this.slaveMotors.length; i++) {
			this.slaveMotors[i].setControl(this.followerRequest.withLeaderID(this.masterMotor.getDeviceID()));
		}
	}

	@Override
	public void setVelocityRadsPerSec(double velocityRadsPerSec) {
		this.masterMotor.setControl(this.velocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
		);
		for (int i = 0; i < this.slaveMotors.length; i++) {
			this.slaveMotors[i].setControl(this.followerRequest.withLeaderID(this.masterMotor.getDeviceID()));
		}
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.masterMotor.setControl(controlRequest);
		for (int i = 0; i < this.slaveMotors.length; i++) {
			this.slaveMotors[i].setControl(controlRequest);
		}
	}

	@Override
	public void configProfile(double maxAccelRadsPerSecSec, double maxJerkRadsPerSecSecSec) {
		this.masterConfig.MotionMagic
			.withMotionMagicAcceleration(Units.radiansToRotations(maxAccelRadsPerSecSec))
			.withMotionMagicJerk(Units.radiansToRotations(maxJerkRadsPerSecSecSec))
		;
	}

	@Override
	public void configFF(FFGains ffGains) {
		ffGains.update(this.masterConfig.Slot0);
	}

	@Override
	public void configPID(PIDGains pidGains) {
		pidGains.update(this.masterConfig.Slot0);
	}

	@Override
	public void configSend() {
		this.masterMotor.getConfigurator().apply(this.masterConfig);
		for (int i = 0; i < this.slaveMotors.length; i++) {
			this.slaveMotors[i].getConfigurator().apply(this.slaveConfigs[i]);
		}
	}
}

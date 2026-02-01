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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class FlywheelsIOTalonFX implements FlywheelsIO {
	private final TalonFX masterMotor = HardwareDevices.shooterMasterMotorID.talonFX();
	private final TalonFX slave1Motor = HardwareDevices.shooterSlave1MotorID.talonFX();

	private final BaseStatusSignal[] refreshSignals;
	private final EncodedMotorStatusSignalCache masterCache;
	private final EncodedMotorStatusSignalCache slave1Cache;

	private final NeutralOut neutralRequest = new NeutralOut();
	private final CoastOut coastRequest = new CoastOut();
	private final StaticBrake brakeRequest = new StaticBrake();
	private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
	private final StrictFollower followerRequest = new StrictFollower(0);

	public FlywheelsIOTalonFX() {
		var driverConfig = new TalonFXConfiguration();
		driverConfig.MotorOutput
			.withNeutralMode(NeutralModeValue.Coast)
			.withInverted(InvertedValue.CounterClockwise_Positive)
		;
		this.masterMotor.getConfigurator().apply(driverConfig);
		this.slave1Motor.getConfigurator().apply(driverConfig);

		this.masterCache = EncodedMotorStatusSignalCache.from(this.masterMotor);
		this.slave1Cache = EncodedMotorStatusSignalCache.from(this.slave1Motor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.masterCache.encoder().position(),
			this.masterCache.encoder().velocity(),
			this.masterCache.motor().appliedVoltage(),
			this.masterCache.motor().statorCurrent(),
			this.masterCache.motor().supplyCurrent(),
			this.masterCache.motor().torqueCurrent(),
			this.masterCache.motor().deviceTemperature(),
			this.slave1Cache.encoder().position(),
			this.slave1Cache.encoder().velocity(),
			this.slave1Cache.motor().appliedVoltage(),
			this.slave1Cache.motor().statorCurrent(),
			this.slave1Cache.motor().supplyCurrent(),
			this.slave1Cache.motor().torqueCurrent(),
			this.slave1Cache.motor().deviceTemperature(),
		};
	}

	@Override
	public void updateInputs(FlywheelsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.masterMotor.updateFrom(this.masterCache);
		inputs.slave1Motor.updateFrom(this.slave1Cache);
	}

	@Override
	public void setVelocityRadsPerSec(double velocityRadsPerSec, double accelerationRadsPerSecSqr, double feedforwardVolts) {
		this.masterMotor.setControl(this.flywheelVelocityRequest
			.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadsPerSecSqr))
			.withFeedForward(feedforwardVolts)
		);
		this.followerRequest.withLeaderID(this.masterMotor.getDeviceID());
		this.slave1Motor.setControl(this.followerRequest);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralRequest, this.coastRequest, this.brakeRequest);
		this.masterMotor.setControl(controlRequest);
		this.slave1Motor.setControl(controlRequest);
	}

	@Override
	public void configPID(PIDConstants pidConstants) {
		var config = new Slot0Configs();
		this.masterMotor.getConfigurator().refresh(config);
		pidConstants.update(config);
		this.masterMotor.getConfigurator().apply(config);
	}
}

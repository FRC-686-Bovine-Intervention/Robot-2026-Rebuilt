package frc.robot.subsystems.intake.rollers;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedMotor.MotorStatusSignalCache;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
	private final TalonFX leftMotor = HardwareDevices.intakeLeftRollerMotorID.talonFX();
	private final TalonFX rightMotor = HardwareDevices.intakeRightRollerMotorID.talonFX();

	private final TalonFXConfiguration leftConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration rightConfig = new TalonFXConfiguration();

	private final MotorStatusSignalCache leftMotorStatusSignalCache;
	private final MotorStatusSignalCache rightMotorStatusSignalCache;

	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] leftMotorConnectedSignals;
	private final BaseStatusSignal[] rightMotorConnectedSignals;

	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final StrictFollower followerRequest = new StrictFollower(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	public IntakeRollersIOTalonFX() {
		this.leftConfig.MotorOutput
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;
		this.rightConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake)
		;

		this.leftMotor.getConfigurator().apply(this.leftConfig);
		this.rightMotor.getConfigurator().apply(this.rightConfig);

		this.leftMotorStatusSignalCache = MotorStatusSignalCache.from(this.leftMotor);
		this.rightMotorStatusSignalCache = MotorStatusSignalCache.from(this.rightMotor);

		this.refreshSignals = new BaseStatusSignal[] {
			this.leftMotorStatusSignalCache.appliedVoltage(),
			this.leftMotorStatusSignalCache.statorCurrent(),
			this.leftMotorStatusSignalCache.supplyCurrent(),
			this.leftMotorStatusSignalCache.torqueCurrent(),
			this.leftMotorStatusSignalCache.deviceTemperature(),
			this.rightMotorStatusSignalCache.appliedVoltage(),
			this.rightMotorStatusSignalCache.statorCurrent(),
			this.rightMotorStatusSignalCache.supplyCurrent(),
			this.rightMotorStatusSignalCache.torqueCurrent(),
			this.rightMotorStatusSignalCache.deviceTemperature(),
		};
		this.leftMotorConnectedSignals = new BaseStatusSignal[] {
			this.leftMotorStatusSignalCache.appliedVoltage(),
			this.leftMotorStatusSignalCache.statorCurrent(),
			this.leftMotorStatusSignalCache.supplyCurrent(),
			this.leftMotorStatusSignalCache.torqueCurrent(),
			this.leftMotorStatusSignalCache.deviceTemperature()
		};
		this.rightMotorConnectedSignals = new BaseStatusSignal[] {
			this.rightMotorStatusSignalCache.appliedVoltage(),
			this.rightMotorStatusSignalCache.statorCurrent(),
			this.rightMotorStatusSignalCache.supplyCurrent(),
			this.rightMotorStatusSignalCache.torqueCurrent(),
			this.rightMotorStatusSignalCache.deviceTemperature()
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(5.0), this.leftMotorStatusSignalCache.getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(5.0), this.rightMotorStatusSignalCache.getStatusSignals());
		this.leftMotor.optimizeBusUtilization();
		this.rightMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IntakeRollersIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.leftMotorConnected = BaseStatusSignal.isAllGood(this.leftMotorConnectedSignals);
		inputs.rightMotorConnected = BaseStatusSignal.isAllGood(this.rightMotorConnectedSignals);
		inputs.leftMotor.updateFrom(this.leftMotorStatusSignalCache);
		inputs.rightMotor.updateFrom(this.rightMotorStatusSignalCache);
	}

	@Override
	public void setVolts(double volts) {
		this.leftMotor.setControl(this.voltageRequest
			.withOutput(volts)
		);
		this.rightMotor.setControl(this.followerRequest.withLeaderID(this.leftMotor.getDeviceID()));
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.leftMotor.setControl(controlRequest);
		this.rightMotor.setControl(controlRequest);
	}
}

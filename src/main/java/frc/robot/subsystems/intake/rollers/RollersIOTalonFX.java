package frc.robot.subsystems.intake.rollers;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedMotor.MotorStatusSignalCache;

public class RollersIOTalonFX implements RollersIO {
    private final TalonFX motor = HardwareDevices.intakeSlamMotorID.talonFX();
    private final MotorStatusSignalCache motorStatusSignalCache;

    private final BaseStatusSignal[] refreshSignals;
    private final BaseStatusSignal[] motorConnectedSignals;

    private final VoltageOut voltageRequest = new VoltageOut(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();
    
    public RollersIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
			.withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            ;
        this.motor.getConfigurator().apply(motorConfig);

        this.motorStatusSignalCache = MotorStatusSignalCache.from(this.motor);

        this.refreshSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature()
        };
        this.motorConnectedSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature()
        };
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.refreshSignals);
        inputs.motorConnected = BaseStatusSignal.isAllGood(motorConnectedSignals);
        inputs.motor.updateFrom(this.motorStatusSignalCache);
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
			.withOutput(volts));
    }

    @Override
	public void stop(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.motor.setControl(controlRequest);
	}
}
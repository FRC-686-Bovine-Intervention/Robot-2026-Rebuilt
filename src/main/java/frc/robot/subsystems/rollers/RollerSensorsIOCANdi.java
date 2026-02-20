package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

import frc.robot.constants.RobotConstants;

public class RollerSensorsIOCANdi implements RollerSensorsIO {
	private final CANdi candi;

	private final StatusSignal<Boolean> sensorStatusSignal;
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] connectedSignals;

	public RollerSensorsIOCANdi(CANdi candi) {
		this.candi = candi;

		var candiConfig = new DigitalInputsConfigs();
		this.candi.getConfigurator().refresh(candiConfig);

		candiConfig
			.withS2FloatState(S2FloatStateValue.FloatDetect)
			.withS2CloseState(S2CloseStateValue.CloseWhenLow)
		;

		this.sensorStatusSignal = this.candi.getS2Closed();
		this.refreshSignals = new BaseStatusSignal[] {
			this.sensorStatusSignal,
		};
		this.connectedSignals = new BaseStatusSignal[] {
			this.sensorStatusSignal,
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.sensorStatusSignal);
		this.candi.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(RollerSensorsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.connected = BaseStatusSignal.isAllGood(this.connectedSignals);

		inputs.feederSensor = this.sensorStatusSignal.getValue();
	}
}

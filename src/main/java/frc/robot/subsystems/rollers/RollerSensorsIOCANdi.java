package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.commonDevices.CommonCANdi;

public class RollerSensorsIOCANdi implements RollerSensorsIO {
	private final StatusSignal<Boolean> sensorStatusSignal;
	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] connectedSignals;

	public RollerSensorsIOCANdi(CommonCANdi candi) {
		candi.candiConfig.DigitalInputs
			.withS2FloatState(S2FloatStateValue.FloatDetect)
			.withS2CloseState(S2CloseStateValue.CloseWhenLow)
		;

		this.sensorStatusSignal = candi.candi.getS2Closed();
		this.refreshSignals = new BaseStatusSignal[] {
			this.sensorStatusSignal,
		};
		this.connectedSignals = new BaseStatusSignal[] {
			this.sensorStatusSignal,
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.sensorStatusSignal);
	}

	@Override
	public void updateInputs(RollerSensorsIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.connected = BaseStatusSignal.isAllGood(this.connectedSignals);

		inputs.feederSensor = this.sensorStatusSignal.getValue();
	}
}

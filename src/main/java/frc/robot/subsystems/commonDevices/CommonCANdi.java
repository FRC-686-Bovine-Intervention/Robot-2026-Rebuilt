package frc.robot.subsystems.commonDevices;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;

import frc.robot.constants.HardwareDevices;

public class CommonCANdi {
	public final CANdi candi = HardwareDevices.candiID.candi();
	public final CANdiConfiguration candiConfig = new CANdiConfiguration();

	public CommonCANdi() {

	}

	public void configSend() {
		this.candi.getConfigurator().apply(this.candiConfig);
		this.candi.optimizeBusUtilization();
	}
}

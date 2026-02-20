package frc.robot.subsystems.commonDevices;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;

import frc.robot.constants.HardwareDevices;

public class CommonCANdi {
	public final CANdi candi = HardwareDevices.candiID.candi();

	public CommonCANdi() {
		var config = new CANdiConfiguration();

		this.candi.getConfigurator().apply(config);
	}
}

package frc.util.hardwareID.rioPorts;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;

public class AIOPort {
	public final int port;

	private AIOPort(int port) {
		this.port = port;
	}

	public static AIOPort port(int port) {
		return new AIOPort(port);
	}

	public AnalogInput input() {
		return new AnalogInput(port);
	}
	public AnalogOutput output() {
		return new AnalogOutput(port);
	}
}

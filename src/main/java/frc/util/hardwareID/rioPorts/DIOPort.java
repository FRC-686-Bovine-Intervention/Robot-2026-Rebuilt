package frc.util.hardwareID.rioPorts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class DIOPort {
	public final int port;

	private DIOPort(int port) {
		this.port = port;
	}

	public static DIOPort port(int port) {
		return new DIOPort(port);
	}

	public DigitalInput input() {
		return new DigitalInput(port);
	}
	public DigitalOutput output() {
		return new DigitalOutput(port);
	}
}

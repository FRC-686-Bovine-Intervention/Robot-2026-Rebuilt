package frc.util.hardwareID.rioPorts;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

public class PWMPort {
	public final int port;

	private PWMPort(int port) {
		this.port = port;
	}

	public static PWMPort port(int port) {
		return new PWMPort(port);
	}

	public PWM channel() {
		return new PWM(port);
	}
	public AddressableLED addressableLED() {
		return new AddressableLED(port);
	}
	public Servo servo() {
		return new Servo(port);
	}
}

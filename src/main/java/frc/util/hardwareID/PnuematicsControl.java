package frc.util.hardwareID;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.util.hardwareID.can.CANDevice;

public class PnuematicsControl {
	public final CANDevice canDevice;
	public final PneumaticsModuleType type;

	public PnuematicsControl(CANDevice canDevice, PneumaticsModuleType type) {
		this.canDevice = canDevice;
		this.type = type;
	}

	public SolenoidPort solenoidPort(int port) {
		return new SolenoidPort(this, port);
	}
	public DoubleSolenoidPorts doubleSolenoidPort(int forwardPort, int backwardPort) {
		return new DoubleSolenoidPorts(this, forwardPort, backwardPort);
	}

	public static class SolenoidPort {
		public final PnuematicsControl device;
		public final int port;

		private SolenoidPort(PnuematicsControl device, int port) {
			this.device = device;
			this.port = port;
		}

		public Solenoid solenoid() {
			return new Solenoid(device.canDevice.id, device.type, port);
		}
	}
	public static class DoubleSolenoidPorts {
		public final PnuematicsControl device;
		public final int forwardPort;
		public final int reversePort;

		private DoubleSolenoidPorts(PnuematicsControl device, int forwardPort, int reversePort) {
			this.device = device;
			this.forwardPort = forwardPort;
			this.reversePort = reversePort;
		}

		public DoubleSolenoid doubleSolenoid() {
			return new DoubleSolenoid(device.canDevice.id, device.type, forwardPort, reversePort);
		}
	}
}

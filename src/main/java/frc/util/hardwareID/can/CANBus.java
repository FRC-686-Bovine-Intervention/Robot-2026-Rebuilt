package frc.util.hardwareID.can;

public class CANBus {
	public final String name;

	private CANBus(String name) {
		this.name = name;
	}

	public static CANBus newBus(String name) {
		return new CANBus(name);
	}
	public static CANBus rio() {
		return newBus("rio");
	}

	public CANDevice id(int id) {
		return CANDevice.id(id, this);
	}
}

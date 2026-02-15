package frc.robot.constants;

import frc.util.hardwareID.can.CANBus;
import frc.util.hardwareID.can.CANDevice;
import frc.util.hardwareID.rioPorts.PWMPort;

public class HardwareDevices {
	/*
	 * PDH Ports
	 * 10:                      9:
	 * 11:                      8:
	 * 12:                      7:
	 * 13:                      6:
	 * 14:                      5:
	 * 15:                      4:
	 * 16:                      3:
	 * 17:                      2:
	 * 18:                      1:
	 * 19:                      0:
	 * 20:
	 * 21:
	 * 22:
	 * 23:
	 */
	public static final CANBus rio = CANBus.rio();
	public static final CANBus canivore = CANBus.newBus("canivore");

	// Drive
	public static final CANDevice pigeonID = canivore.id(0);
	// | Front Left
	public static final CANDevice frontLeftDriveMotorID = canivore.id(1);
	public static final CANDevice frontLeftAzimuthMotorID = rio.id(1);
	// | Front Right
	public static final CANDevice frontRightDriveMotorID = canivore.id(2);
	public static final CANDevice frontRightAzimuthMotorID = rio.id(2);
	// | Back Left
	public static final CANDevice backLeftDriveMotorID = canivore.id(3);
	public static final CANDevice backLeftAzimuthMotorID = rio.id(3);
	// | Back Right
	public static final CANDevice backRightDriveMotorID = canivore.id(4);
	public static final CANDevice backRightAzimuthMotorID = rio.id(4);

	// RIO
	public static final PWMPort ledPort = PWMPort.port(0);
}

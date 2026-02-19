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
	public static final CANDevice frontLeftTurnMotorID = rio.id(1);
	// | Front Right
	public static final CANDevice frontRightDriveMotorID = canivore.id(2);
	public static final CANDevice frontRightTurnMotorID = rio.id(2);
	// | Back Left
	public static final CANDevice backLeftDriveMotorID = canivore.id(3);
	public static final CANDevice backLeftTurnMotorID = rio.id(3);
	// | Back Right
	public static final CANDevice backRightDriveMotorID = canivore.id(4);
	public static final CANDevice backRightTurnMotorID = rio.id(4);


	// Intake
	// | Slam
	public static final CANDevice intakeSlamMotorID = canivore.id(5);
	public static final CANDevice intakeSlamEncoderID = canivore.id(6);
	// | Rollers
	public static final CANDevice intakeRollerMotorID = canivore.id(7);

	// Rollers
	// | Indexer
	public static final CANDevice indexerMotorID = canivore.id(8);
	// | Agitator
	public static final CANDevice agitatorMotorID = canivore.id(9);
	// | Feeder
	public static final CANDevice feederMotorID = canivore.id(10);

	// Shooter
	// | Flywheels
	// | | Left Flywheel System
	public static final CANDevice leftFlywheelMotorMasterID = canivore.id(5);
	public static final CANDevice leftFlywheelMotorSlaveID = canivore.id(6);
	// | | Right Flywheel System
	public static final CANDevice rightFlywheelMotorMasterID = canivore.id(9);
	public static final CANDevice rightFlywheelMotorSlaveID = canivore.id(10);
	// | Hood
	public static final CANDevice hoodMotorID = canivore.id(11);

	// RIO
	public static final PWMPort ledPort = PWMPort.port(0);
}

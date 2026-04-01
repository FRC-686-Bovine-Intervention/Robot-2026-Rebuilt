package frc.robot.constants;

import frc.util.hardwareID.can.CANBus;
import frc.util.hardwareID.can.CANDevice;
import frc.util.hardwareID.rioPorts.PWMPort;

public class HardwareDevices {
	/*
	 * PDP Ports
	 * 12: FL Drive          11: FR Drive
	 * 13: FL Azimuth        10: FR Azimuth
	 * 14: L Intake Roller    9: R Intake Roller
	 * 15: Intake Slam        8: MPM
	 * 16: Radio              7: Rio
	 * 17: Radio              6: Climber
	 * 18: Indexer            5: Hood
	 * 19: Agitator           4: Feeder
	 * 20: LS Flywheel        3: RS Flywheel
	 * 21: LM Flywheel        2: RM Flywheel
	 * 22: BL Azimuth         1: BR Azimuth
	 * 23: BL Drive           0: BR Drive
	 */
	public static final CANBus rio = CANBus.rio();
	public static final CANBus canivore = CANBus.newBus("canivore");

	// Drive
	public static final CANDevice pigeonID = rio.id(0);
	// | Front Left
	public static final CANDevice frontLeftDriveMotorID = rio.id(1);
	public static final CANDevice frontLeftAzimuthMotorID = rio.id(1);
	// | Front Right
	public static final CANDevice frontRightDriveMotorID = rio.id(2);
	public static final CANDevice frontRightAzimuthMotorID = rio.id(2);
	// | Back Left
	public static final CANDevice backLeftDriveMotorID = rio.id(3);
	public static final CANDevice backLeftAzimuthMotorID = rio.id(3);
	// | Back Right
	public static final CANDevice backRightDriveMotorID = rio.id(4);
	public static final CANDevice backRightAzimuthMotorID = rio.id(4);

	// Intake
	// | Slam
	public static final CANDevice intakeSlamMotorID = rio.id(5);
	public static final CANDevice intakeSlamEncoderID = rio.id(5);
	// | Rollers
	public static final CANDevice intakeLeftRollerMotorID = rio.id(6);
	public static final CANDevice intakeRightRollerMotorID = rio.id(7);

	// Rollers
	// | Indexer
	public static final CANDevice indexerMotorID = rio.id(8);
	// | Agitator
	public static final CANDevice agitatorMotorID = rio.id(9);
	// | Feeder
	public static final CANDevice feederMotorID = rio.id(10);

	// Shooter
	// | Hood
	public static final CANDevice hoodMotorID = rio.id(11);
	// | Flywheels
	// | | Left Flywheel System
	public static final CANDevice leftBottomFlywheelMotorID = rio.id(12);
	public static final CANDevice leftTopFlywheelMotorID = rio.id(13);
	// | | Right Flywheel System
	public static final CANDevice rightBottomFlywheelMotorID = rio.id(14);
	public static final CANDevice rightTopFlywheelMotorID = rio.id(15);

	// Climber
	// | Hook
	public static final CANDevice climberHookMotorID = rio.id(16);

	// CANdi
	public static final CANDevice candiID = rio.id(0);

	// RIO
	public static final PWMPort ledPort = PWMPort.port(0);
}

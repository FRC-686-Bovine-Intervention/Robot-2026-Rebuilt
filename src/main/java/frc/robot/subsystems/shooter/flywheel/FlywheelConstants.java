package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.constants.HardwareDevices;
import frc.util.hardwareID.can.CANDevice;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;
import lombok.RequiredArgsConstructor;

public class FlywheelConstants {
	// Gear Ratios
	public static final GearRatio driverMotorToFlywheelRatio = new GearRatio()

	;
	// Flywheel wheel
	public static final LinearRelation driverFlywheelWheel = LinearRelation.wheelDiameter(Inches.of(4));

	// Flywheel configuration
	@RequiredArgsConstructor
	public static class FlywheelConfig {
		public final String name;
		public final CANDevice masterMotorID;
		public final CANDevice slaveMotorID;
	}

	public static final FlywheelConfig leftFlywheelConfig = new FlywheelConfig(
		"Left",
		HardwareDevices.leftFlywheelMotorMasterID,
		HardwareDevices.leftFlywheelMotorSlaveID
	);
	public static final FlywheelConfig rightFlywheelConfig = new FlywheelConfig(
		"Right",
		HardwareDevices.rightFlywheelMotorMasterID,
		HardwareDevices.rightFlywheelMotorSlaveID
	);
}

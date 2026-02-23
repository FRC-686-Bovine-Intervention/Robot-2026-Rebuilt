package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.constants.HardwareDevices;
import frc.util.hardwareID.can.CANDevice;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;
import lombok.RequiredArgsConstructor;

public class FlywheelConstants {
	// Gear Ratios
	public static final GearRatio motorToMechanism = new GearRatio()
	.sprocket(24)
	.axle()
	.sprocket(16)
	.axle()
	;

	public static final GearRatio flywheelToHood = new GearRatio()
	.sprocket(38)
	.axle()
	.sprocket(16)
	.axle()
	;
	// Flywheel wheel
	public static final LinearRelation wheel = LinearRelation.wheelDiameter(Inches.of(3.0));
	public static final LinearRelation hoodRoller = LinearRelation.wheelDiameter(Inches.of(1.0));

	// Flywheel configuration
	@RequiredArgsConstructor
	public static class FlywheelConfig {
		public final String name;
		public final CANDevice masterMotorID;
		public final CANDevice slaveMotorID;
		public final InvertedValue masterInvertedValue;
		public final InvertedValue slaveInvertedValue;
	}

	public static final FlywheelConfig leftFlywheelConfig = new FlywheelConfig(
		"Left",
		HardwareDevices.leftFlywheelMotorMasterID,
		HardwareDevices.leftFlywheelMotorSlaveID,
		InvertedValue.CounterClockwise_Positive,
		InvertedValue.CounterClockwise_Positive
	);
	public static final FlywheelConfig rightFlywheelConfig = new FlywheelConfig(
		"Right",
		HardwareDevices.rightFlywheelMotorMasterID,
		HardwareDevices.rightFlywheelMotorSlaveID,
		InvertedValue.Clockwise_Positive,
		InvertedValue.Clockwise_Positive
	);
}

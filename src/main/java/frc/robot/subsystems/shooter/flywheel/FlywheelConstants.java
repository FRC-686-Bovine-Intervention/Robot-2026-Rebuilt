package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;

import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public class FlywheelConstants {
	// Gear Ratios
	public static final GearRatio motorToMechanism = new GearRatio()
		.sprocket(20)
		.sprocket(24)
	;

	public static final GearRatio flywheelToHood = new GearRatio()
		.sprocket(38)
		.sprocket(16)
		.gear(24).gear(24).axle()
	;

	// Flywheel wheel
	public static final LinearRelation wheel = LinearRelation.wheelDiameter(Inches.of(3.0));
	public static final LinearRelation hoodRoller = LinearRelation.wheelDiameter(Inches.of(1.1));
}

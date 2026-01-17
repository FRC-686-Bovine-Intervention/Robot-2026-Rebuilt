package frc.robot.subsystems.shooter.flywheels;

import static edu.wpi.first.units.Units.Inches;

import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public class FlywheelsConstants {
	public static final LinearRelation driverFlywheelWheel = LinearRelation.wheelDiameter(Inches.of(3));
	public static final LinearRelation kickerWheel = LinearRelation.wheelDiameter(Inches.of(2));

	public static final GearRatio driverMotorToFlywheelRatio = new GearRatio()

	;
	public static final GearRatio kickerMotorToFlywheelRatio = new GearRatio()

	;
}

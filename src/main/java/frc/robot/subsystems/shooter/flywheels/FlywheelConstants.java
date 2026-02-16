package frc.robot.subsystems.shooter.flywheels;

import static edu.wpi.first.units.Units.Inches;

import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public class FlywheelConstants {
	public static final LinearRelation driverFlywheelWheel = LinearRelation.wheelDiameter(Inches.of(4));

	public static final GearRatio driverMotorToFlywheelRatio = new GearRatio()

	;
}

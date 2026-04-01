package frc.robot.subsystems.climber.hook;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public class HookConstants {
	// Physical Limits
	public static final Distance minLength = Inches.of(0.0);
	public static final Distance maxLength = Inches.of(9.0);

	// Sensor Offsets
	public static final Distance limitSwitchLength = Inches.of(0.0);

	// Gear Ratios
	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(5)
		.planetary(5)
	;
	// Rope spool
	public static final LinearRelation spool = LinearRelation.wheelDiameter(Inches.of(0.91));

	// Ascope Mech Constants
	public static final Transform3d hookBase = new Transform3d(
		new Translation3d(
			Meters.of(-0.039076),
			Meters.of(-0.323914),
			Meters.of(+0.184150)
		),
		Rotation3d.kZero
	);
}

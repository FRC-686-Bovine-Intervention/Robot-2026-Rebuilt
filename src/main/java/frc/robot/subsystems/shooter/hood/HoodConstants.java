package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.geometry.GeomUtil;
import frc.util.mechanismUtil.GearRatio;

public class HoodConstants {
	// Physical Limits
	public static final Angle minAngle = new Translation2d(
		Inches.of(7.045389),
		Inches.of(1.261467)
	).getAngle().getMeasure();
	public static final Angle maxAngle = minAngle.plus(Degrees.of(35.0));

	// Sensor Offsets
	public static final Angle limitSwitchAngle = minAngle;

	// Gear Ratios
	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(GearRatio.ULTRAPLANETARY_3_1)
		.planetary(GearRatio.ULTRAPLANETARY_3_1)
		.gear(10).gear(160).axle()
	;


	// Ascope Mech Constants
	public static final Transform3d hoodBase = new Transform3d(
		new Translation3d(
			Meters.of(-0.103427),
			Meters.of(0),
			Meters.of(0.492469)
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
}

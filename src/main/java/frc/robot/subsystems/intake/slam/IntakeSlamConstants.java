package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.geometry.GeomUtil;
import frc.util.mechanismUtil.GearRatio;

public class IntakeSlamConstants {
	// Physical Limits
	public static final Angle minAngle = Degrees.of(0.0);
	public static final Angle maxAngle = Degrees.of(150.0);

	// Sensor Offsets
	public static final Angle encoderZeroOffset = Degrees.of(0.0);

	// Gear Ratios
	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(5.0)
		.planetary(5.0)
		.gear(44).gear(72).axle()
		.sprocket(18).sprocket(36)
	;
	public static final GearRatio sensorToMechanism = new GearRatio()
		.sprocket(18).sprocket(36)
	;


	// Ascope Mech Constants
	public static final Translation2d mechBase2d = new Translation2d(
		Meters.of(+0.307975),
		Meters.of(+0.241300)
	);
	public static final Transform3d mechBase3d = new Transform3d(
		new Translation3d(
			mechBase2d.getMeasureX(),
			Inches.of(0.0),
			mechBase2d.getMeasureY()
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
}

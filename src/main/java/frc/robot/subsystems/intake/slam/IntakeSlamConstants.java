package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
		.sprocket(16).sprocket(36)
	;
	public static final GearRatio sensorToMechanism = new GearRatio()
		.sprocket(16).sprocket(36)
	;


	// Ascope Mech Constants
	public static final Translation2d mechDriverAxle2d = new Translation2d(
		Inches.of(+6.991450),
		Inches.of(+8.000000)
	);
	public static final Translation2d mechFollowerAxle2d = new Translation2d(
		Inches.of(+4.546233),
		Inches.of(+9.691762)
	);
	public static final Distance mechDriverLength = Inches.of(16.455404);
	public static final Distance mechFollowerLength = Inches.of(16.455404);
	public static final Distance mechCouplerLength = Inches.of(2.973406);
	public static final Distance mechBaseLength = Meters.of(mechDriverAxle2d.getDistance(mechFollowerAxle2d));
	public static final Transform3d mechDriverBase3d = new Transform3d(
		new Translation3d(
			mechDriverAxle2d.getMeasureX(),
			Inches.of(0.0),
			mechDriverAxle2d.getMeasureY()
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
	public static final Transform3d mechFollowerBase3d = new Transform3d(
		new Translation3d(
			mechFollowerAxle2d.getMeasureX(),
			Inches.of(0.0),
			mechFollowerAxle2d.getMeasureY()
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
	public static final Transform3d mechCouplerBase3d = new Transform3d(
		new Translation3d(
			mechDriverLength,
			Inches.of(0.0),
			Inches.of(0.0)
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
}

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.geometry.GeomUtil;

public final class VisionConstants {
	public static final Transform3d bottomLeftMount = new Transform3d(
		new Translation3d(
			Inches.of(-10.100164),
			Inches.of(+12.625000),
			Inches.of(+17.625000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(35))
			.pitch(Degrees.of(-5))
		.build()
	);
	public static final Transform3d topLeftMount = new Transform3d(
		new Translation3d(
			Inches.of(-10.035471),
			Inches.of(+12.375000),
			Inches.of(+20.250000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(-5.374629))
			.pitch(Degrees.of(-20))
		.build()
	);

	public static final Transform3d bottomRightMount = new Transform3d(
		new Translation3d(
			Inches.of(-12.956588),
			Inches.of(-10.514151),
			Inches.of(+11.000000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(180))
			.yaw(Degrees.of(+11.904820))
		.build()
	);

	public static final Transform3d topRightMount = new Transform3d(
		new Translation3d(
			Inches.of(-9.975120),
			Inches.of(-12.765165),
			Inches.of(+14.000000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(-45.000000))
			.pitch(Degrees.of(-12.000000))
		.build()
	);

	public static final Transform3d intakeMount = new Transform3d(
		new Translation3d(
			Inches.of(-9.975120),
			Inches.of(-12.765165),
			Inches.of(+14.000000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(-45.000000))
			.pitch(Degrees.of(-12.000000))
		.build()
	);
}

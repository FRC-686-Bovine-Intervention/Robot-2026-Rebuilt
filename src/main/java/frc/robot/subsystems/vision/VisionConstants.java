package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.geometry.GeomUtil;

public final class VisionConstants {
	public static final Transform3d frontLeftMount = new Transform3d(
		new Translation3d(
			Inches.of(+7.953314),
			Inches.of(+11.042047),
			Inches.of(+9.617869)
		),
		GeomUtil.rotation3dBuilder()
			.pitch(Degrees.of(-27.5))
		.build()
	);
	public static final Transform3d frontRightMount = new Transform3d(
		new Translation3d(
			Inches.of(+7.953314),
			Inches.of(-11.042047),
			Inches.of(+9.617869)
		),
		GeomUtil.rotation3dBuilder()
			.pitch(Degrees.of(-27.5))
		.build()
	);

	public static final Transform3d intakeMount = new Transform3d(
		new Translation3d(
			Inches.of(-3.896877),
			Inches.of(-6.210415),
			Inches.of(+21.633440)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(180))
			.pitch(Degrees.of(+19))
			.yaw(Degrees.of(-9))
		.build()
	);
}

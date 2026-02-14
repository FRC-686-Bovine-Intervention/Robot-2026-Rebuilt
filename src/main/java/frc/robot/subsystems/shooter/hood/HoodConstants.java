package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.geometry.GeomUtil;
import frc.util.mechanismUtil.GearRatio;

public class HoodConstants {
	public static final GearRatio motorToMechanism = new GearRatio(1);

	public static final Angle maxAngle = Degrees.of(30);
	public static final Angle minAngle = Degrees.of(0);

	public static final Transform3d hoodBase = new Transform3d(
		new Translation3d(

		),
		GeomUtil.rotation3dBuilder().yaw(0).build()
	);
}
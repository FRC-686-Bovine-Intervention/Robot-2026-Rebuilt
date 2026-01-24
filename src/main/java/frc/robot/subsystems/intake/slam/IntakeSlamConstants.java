package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.geometry.GeomUtil;
import frc.util.mechanismUtil.GearRatio;

public class IntakeSlamConstants {
	public static final Angle minAngle = Degrees.of(0);
	public static final Angle maxAngle = Degrees.of(67);

	public static final Angle cancoderZeroOffset = Degrees.of(67);

	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(5)
		.planetary(5)
		.sprocket(20).sprocket(20)
		.sprocket(20).sprocket(20)
	;
	public static final GearRatio sensorToMechanism = new GearRatio()

	;

	public static final Translation2d primaryDriverAxleRobotSpace = new Translation2d(
		Inches.of(+6.559830),
		Inches.of(+9.550000)
	);
	public static final Translation2d primaryFollowerAxleRobotSpace = new Translation2d(
		Inches.of(+10.140892),
		Inches.of(+7.550000)
	);
	public static final Distance primaryFrameLength = Meters.of(primaryDriverAxleRobotSpace.getDistance(primaryFollowerAxleRobotSpace));
	public static final Angle primaryFrameNormalAngle = primaryFollowerAxleRobotSpace.minus(primaryDriverAxleRobotSpace).getAngle().plus(Rotation2d.kCCW_90deg).getMeasure();

	public static final Distance primaryDriverLength = Inches.of(12.101226);
	public static final Distance primaryFollowerLength = Inches.of(10.609151);
	public static final Distance primaryCouplerLength = Inches.of(3.747302);

	public static final Angle primaryCouplerToSecondaryDriverOffset = Degrees.of(0.0);
	public static final Translation2d secondaryDriverAxlePrimaryDriverSpace = new Translation2d(
		primaryDriverLength,
		Inches.of(+0)
	);
	public static final Translation2d secondaryFollowerAxlePrimaryDriverSpace = new Translation2d(
		primaryDriverLength.minus(Inches.of(+2.1073042989091537465211014971286)),
		Inches.of(0.635000)
	);
	public static final Distance secondaryFrameLength = Meters.of(secondaryDriverAxlePrimaryDriverSpace.getDistance(secondaryFollowerAxlePrimaryDriverSpace));
	public static final Angle secondaryFrameNormalAngle = secondaryFollowerAxlePrimaryDriverSpace.minus(secondaryDriverAxlePrimaryDriverSpace).getAngle().plus(Rotation2d.kCCW_90deg).getMeasure();

	public static final Distance secondaryDriverLength = Inches.of(7.394853);
	public static final Distance secondaryFollowerLength = Inches.of(10.401073);
	public static final Distance secondaryCouplerLength = Inches.of(4.950000);

	public static final boolean calibrationSensorInverted = false;

	public static final Transform3d cadOrigin = new Transform3d(new Translation3d(
		-0.016920, 0.001222, 0.123726
	),Rotation3d.kZero);

	public static final Transform3d primaryDriverBase = new Transform3d(
		new Translation3d(
			Inches.of(-6.559830),
			Inches.of(+0),
			Inches.of(+9.550000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(180))
			.pitch(Degrees.of(80.052205))
		.build()
	);
	public static final Transform3d primaryFollowerBase = new Transform3d(
		new Translation3d(
			Inches.of(-10.140892),
			Inches.of(+0),
			Inches.of(+7.550000)
		),
		GeomUtil.rotation3dBuilder()
			.yaw(Degrees.of(180))
			.pitch(Degrees.of(87.881550))
		.build()
	);
	public static final Transform3d primaryCouplerBase = new Transform3d(
		new Translation3d(
			primaryDriverLength,
			Inches.of(+0),
			Inches.of(+0)
		),
		GeomUtil.rotation3dBuilder()
			.pitch(Degrees.of(180).minus(Degrees.of(40.395547)))
		.build()
	);
	public static final Transform3d secondaryFollowerBase = new Transform3d(
		new Translation3d(
			primaryDriverLength.minus(Inches.of(+2.1073042989091537465211014971286)),
			Inches.of(+0),
			Inches.of(0.635000)
		),
		GeomUtil.rotation3dBuilder()
			.pitch(Degrees.of(180).minus(Degrees.of(35.892570)))
		.build()
	);
	public static final Transform3d secondaryCouplerBase = new Transform3d(
		new Translation3d(
			secondaryDriverLength,
			Inches.of(+0),
			Inches.of(0)
		),
		GeomUtil.rotation3dBuilder()
			.pitch(Degrees.of(180).minus(Degrees.of(147.424329)))
		.build()
	);
}

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipped;
import frc.util.geometry.PoseBoundingBoxUtil.BoundingBox;
import frc.util.geometry.PoseBoundingBoxUtil.OrBox;
import frc.util.geometry.PoseBoundingBoxUtil.RectangularBoundingBox;
import frc.util.geometry.PoseBoundingBoxUtil.VerticalLine;
import frc.util.misc.ApriltagUtil.AprilTagPair;

public final class FieldConstants {
	public static final Distance fieldLength = Inches.of(651.2);
	public static final Distance fieldWidth =  Inches.of(317.7);

	// public static final AllianceFlipped<VerticalLine> allianceBox = AllianceFlipped.fromBlue(new VerticalLine(fieldLength.div(2.0).in(Meters), false));

	public static final AprilTagFieldLayout apriltagLayout;
	static {
		AprilTagFieldLayout a = null;
		try {
			a = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		} catch(Exception e) {
			e.printStackTrace();
		}
		apriltagLayout = a;
	}

	public static final Distance blueAllianceZoneBoundaryX = Inches.of(158.600000);
	private static final Distance allianceTapeWidth = Inches.of(2);
	public static final AllianceFlipped<Distance> robotStartingLineCenterX = AllianceFlipped.fromBlueXPos(blueAllianceZoneBoundaryX.minus(allianceTapeWidth.div(2.0)));
	public static final AllianceFlipped<Distance> barrierCenterX = AllianceFlipped.fromBlueXPos(Inches.of(182.100000));

	public static final AllianceFlipped<Translation2d> hubCenter = AllianceFlipped.fromBlue(new Translation2d(
		barrierCenterX.getBlue(),
		fieldWidth.div(2.0)
	));
	public static final Distance hubHeight = Inches.of(72.000000);

	public static final AllianceFlipped<Translation2d> towerRungsCenter = AllianceFlipped.fromBlue(new Translation2d(
		Inches.of(41.798750),
		Inches.of(147.475)
	));
	public static final Distance towerInnerUprightDistance = Inches.of(32.250000);

	public static final AllianceFlipped<Distance> outpostCenterY = AllianceFlipped.fromBlueYPos(Inches.of(26.150500));

	public static final AllianceFlipped<VerticalLine> allianceZone = AllianceFlipped.fromBlue(new VerticalLine(blueAllianceZoneBoundaryX.in(Meters), false));

	public static final Angle bumpAngle = Degrees.of(15.0);
	public static final Distance bumpHeight = Inches.of(6.5);
	public static final Distance bumpWidth = Inches.of(73.000000);
	public static final Distance bumpLength = bumpHeight.div(Math.tan(bumpAngle.in(Radians))).times(2.0);

	public static final Distance hubWidth = Inches.of(47.0);

	private static final Distance bottomBumpTopY = fieldWidth.div(2.0).minus(hubWidth.div(2.0));
	private static final Distance bottomBumpBottomY = bottomBumpTopY.minus(bumpWidth);
	private static final Distance topBumpTopY = fieldWidth.div(2.0).plus(hubWidth.div(2.0));
	private static final Distance topBumpBottomY = topBumpTopY.plus(bumpWidth);

	public static final AllianceFlipped<RectangularBoundingBox> bottomBump = AllianceFlipped.fromBlue(BoundingBox.rectangle(
		new Translation2d(
			barrierCenterX.getBlue().minus(bumpLength.div(2.0)),
			bottomBumpBottomY
		),
		new Translation2d(
			barrierCenterX.getBlue().plus(bumpLength.div(2.0)),
			bottomBumpTopY
		)
	));
	public static final AllianceFlipped<RectangularBoundingBox> topBump = AllianceFlipped.fromBlue(BoundingBox.rectangle(
		new Translation2d(
			barrierCenterX.getBlue().minus(bumpLength.div(2.0)),
			topBumpBottomY
		),
		new Translation2d(
			barrierCenterX.getBlue().plus(bumpLength.div(2.0)),
			topBumpTopY
		)
	));
	public static final AllianceFlipped<OrBox> anyBump = AllianceFlipped.fromFunction((alliance) -> bottomBump.get(alliance).or(topBump.get(alliance)));

	public static final AllianceFlipped<AprilTagPair> hubLeft = new AllianceFlipped<AprilTagPair>(
		new AprilTagPair(21, 24),
		new AprilTagPair(5, 8)
	);

	public static final AllianceFlipped<AprilTagPair> hubRight = new AllianceFlipped<AprilTagPair>(
		new AprilTagPair(18, 27),
		new AprilTagPair(2, 11)
	);

	public static final AllianceFlipped<AprilTagPair> hubFront = new AllianceFlipped<AprilTagPair>(
		new AprilTagPair(26, 25),
		new AprilTagPair(10, 9)
	);

	public static final AllianceFlipped<AprilTagPair> hubRear = new AllianceFlipped<AprilTagPair>(
		new AprilTagPair(20, 19),
		new AprilTagPair(4, 3)
	);
}

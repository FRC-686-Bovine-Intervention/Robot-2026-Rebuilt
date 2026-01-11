package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipped;
import frc.util.geometry.PoseBoundingBoxUtil.VerticalLine;

public final class FieldConstants {
	// From AndyMark field CAD, wall base to wall base
	public static final Distance fieldLength = Inches.of(650.118760);
	public static final Distance fieldWidth =  Inches.of(316.641000);

	public static final AllianceFlipped<VerticalLine> allianceBox = AllianceFlipped.fromBlue(new VerticalLine(fieldLength.div(2.0).in(Meters), false));

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

	public static final AllianceFlipped<Distance> robotStartingLineCenterX = AllianceFlipped.fromBlueXPos(Inches.of(157.059380));
	public static final AllianceFlipped<Distance> barrierLineCenterX = AllianceFlipped.fromBlueXPos(Inches.of(189.816986));

	public static final AllianceFlipped<Translation2d> hubCenter = AllianceFlipped.fromBlue(new Translation2d(
		barrierLineCenterX.getBlue(),
		Inches.of(158.320500)
	));
	public static final Distance hubHeight = Inches.of(72.000000);

	public static final AllianceFlipped<Translation2d> towerRungsCenter = AllianceFlipped.fromBlue(new Translation2d(
		Inches.of(42.047000),
		Inches.of(146.858)
	));
	public static final Distance towerInnerUprightDistance = Inches.of(32.250000);

	public static final AllianceFlipped<Distance> outpostCenterY = AllianceFlipped.fromBlueYPos(Inches.of(25.621000));
}

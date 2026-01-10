package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipped;
import frc.util.geometry.PoseBoundingBoxUtil.VerticalLine;

public final class FieldConstants {
	public static final Distance fieldLength = Inches.of(648.000000);
	public static final Distance fieldWidth =  Inches.of(324.00000);

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
}

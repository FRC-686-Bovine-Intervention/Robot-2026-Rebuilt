package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;
import frc.util.geometry.PoseBoundingBoxUtil.BoundingBox;
import frc.util.geometry.PoseBoundingBoxUtil.RectangularBoundingBox;

public class ApriltagVisionConstants {
	public static final Distance zMargin = Inches.of(18.0);
	public static final RectangularBoundingBox acceptableFieldBox = BoundingBox.rectangle(
		new Translation2d(

		),
		new Translation2d(
			FieldConstants.fieldLength,
			FieldConstants.fieldWidth
		)
	);
}

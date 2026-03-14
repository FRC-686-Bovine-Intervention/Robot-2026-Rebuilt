package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
	public static final Time allottedAutoTime = Seconds.of(15.3);
	public static final Time disabledTime = Seconds.of(3.0);

	public static final AllianceFlipped<Pose2d> startHubFront = FieldConstants.hubFrontRobotPose;

	public static final AllianceFlipped<Pose2d> startInLeftTrench = AllianceFlipped.fromBlue(
		new Pose2d(
			new Translation2d(

			),
			Rotation2d.k180deg
		)
	);

	public static final AllianceFlipped<Pose2d> startInRightTrench = AllianceFlipped.fromBlue(
		new Pose2d(
			new Translation2d(

			),
			Rotation2d.k180deg
		)
	);
}

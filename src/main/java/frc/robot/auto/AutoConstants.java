package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
	public static final Time allottedAutoTime = Seconds.of(20.3);
	public static final Time disabledTime = Seconds.of(3.0);

	public static final AllianceFlipped<Pose2d> startHubFront = FieldConstants.hubFrontRobotPose;

	public static final AllianceFlipped<Pose2d> startInLeftTrench = AllianceFlipped.fromBlue(
		new Pose2d(
			new Translation2d(
				FieldConstants.robotStartingLineCenterX.getBlue().plus(RobotConstants.centerToFrontBumper),
				FieldConstants.fieldWidth.minus(Inches.of(24.0))
			),
			Rotation2d.k180deg
		)
	);

	public static final AllianceFlipped<Pose2d> startInRightTrench = AllianceFlipped.fromBlue(
		new Pose2d(
			new Translation2d(
				FieldConstants.robotStartingLineCenterX.getBlue().plus(RobotConstants.centerToFrontBumper),
				Inches.of(24.0)
			),
			Rotation2d.k180deg
		)
	);
}

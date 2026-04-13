package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
	public static final Time allottedAutoTime = Seconds.of(20.3);
	public static final Time disabledTime = Seconds.of(4.0);

	public static final Distance startLineX = FieldConstants.robotStartingLineCenterX.getBlue();
	public static final Distance startXInAllianceZone = startLineX.minus(RobotConstants.centerToFrontBumper);
	public static final Distance startXInTrench = startLineX.plus(RobotConstants.centerToFrontBumper);


	public static final AllianceFlipped<Pose2d> startOutsideLeftTrench =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInAllianceZone,
			FieldConstants.fieldWidth.minus(RobotConstants.centerToFrontBumper),
			Rotation2d.k180deg
	));
	public static final AllianceFlipped<Pose2d> startOutsideRightTrench =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInAllianceZone,
			RobotConstants.centerToFrontBumper,
			Rotation2d.k180deg
	));
	public static final AllianceFlipped<Pose2d> startLeftBump =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInAllianceZone,
			FieldConstants.topBumpTopY.plus(FieldConstants.topBumpBottomY).div(2),
			Rotation2d.k180deg
	));
	public static final AllianceFlipped<Pose2d> startRightBump =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInAllianceZone,
			FieldConstants.bottomBumpBottomY.plus(FieldConstants.bottomBumpTopY).div(2),
			Rotation2d.k180deg
	));
	public static final AllianceFlipped<Pose2d> startCenter =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInAllianceZone,
			FieldConstants.fieldWidth.div(2),
			Rotation2d.k180deg
	));

	public static final AllianceFlipped<Pose2d> startInsideLeftTrench =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInTrench,
			FieldConstants.fieldWidth.minus(RobotConstants.centerToFrontBumper),
			Rotation2d.kCW_90deg
	));
	public static final AllianceFlipped<Pose2d> startInsideRightTrench =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInTrench,
			RobotConstants.centerToFrontBumper,
			Rotation2d.kCCW_90deg
	));

	public static final AllianceFlipped<Pose2d> startInsideLeftTrenchDS =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInTrench,
			FieldConstants.fieldWidth.minus(RobotConstants.centerToFrontBumper),
			Rotation2d.k180deg
	));
	public static final AllianceFlipped<Pose2d> startInsideRightTrenchDS =
		AllianceFlipped.fromBlue(new Pose2d(
			startXInTrench,
			RobotConstants.centerToFrontBumper,
			Rotation2d.k180deg
	));

	public static final AllianceFlipped<Pose2d> startInRightTrench = AllianceFlipped.fromBlue(
		new Pose2d(
			new Translation2d(
				FieldConstants.robotStartingLineCenterX.getBlue().plus(RobotConstants.centerToFrontBumper),
				Inches.of(24.0)
			),
			Rotation2d.k180deg
		)
	);

	public enum StartingPosition {
		INSIDE_LEFT_TRENCH(startInsideLeftTrench, "SILT", "SIRT"),
		// LEFT_BUMP(startLeftBump, "SLB", "SRB"),
		CENTER(startCenter, "SC"),
		// RIGHT_BUMP(startRightBump, "SRB", "SLB"),
		INSIDE_RIGHT_TRENCH(startInsideRightTrench, "SIRT", "SILT");

		public final AllianceFlipped<Pose2d> pose;
		public final String alias;
		public final String rightAlias;

		StartingPosition(AllianceFlipped<Pose2d> pose, String alias) {
			this.pose = pose;
			this.alias = alias;
			this.rightAlias = alias;
		}

		StartingPosition(AllianceFlipped<Pose2d> pose, String alias, String rightAlias) {
			this.pose = pose;
			this.alias = alias;
			this.rightAlias = rightAlias;
		}
	}
	public enum IntakeLocation {
		FULL_OUTER_SWIPE("FOS",1),
		FULL_INNER_SWIPE("FIS", 1),
		HALF_OUTER_SWIPE("HOS", 1),
		HALF_INNER_SWIPE("HIS", 1),
		HALF_INNER_LOOP_SWIPE("HILS", 1),
		OPPONENT_SWIPE("OpS", 1),
		HALF_OPPONENT_SWIPE("HOpS", 1),
		HALF_SWEEP("HSw", 1),
		DEPOT("Depot", 0),
		FALSE("False", 1);

		public final String alias;
		public final int canFlip;

		IntakeLocation(String alias, int canFlip) {
			this.alias = alias;
			this.canFlip = canFlip;
		}
	}

	public enum ScoringLocation {
		OUTSIDE_LEFT_TRENCH("OLT", "ORT", 1),
		OUTSIDE_RIGHT_TRENCH("ORT", "OLT", 1),
		CENTER("Center", 1),
		LEFT("Left", "Right", 1),
		RIGHT("Right", "Left", 1),
		STOP("Stop", 1);

		public final String alias;
		public final String rightAlias;
		public final int canFlip;

		ScoringLocation(String alias, int canFlip) {
			this.alias = alias;
			this.rightAlias = alias;
			this.canFlip = canFlip;
		}

		ScoringLocation(String alias, String rightAlias, int canFlip) {
			this.alias = alias;
			this.rightAlias = rightAlias;
			this.canFlip = canFlip;
		}
	}
}

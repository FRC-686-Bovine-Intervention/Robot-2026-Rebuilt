package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(20.3);
    public static final Time disabledTime = Seconds.of(3);

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
            startXInAllianceZone,
            FieldConstants.fieldWidth.minus(RobotConstants.centerToFrontBumper),
            Rotation2d.kZero
    ));
    public static final AllianceFlipped<Pose2d> startInsideRightTrench =
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            RobotConstants.centerToFrontBumper,
            Rotation2d.kZero
    ));


	public enum StartingPosition {
		INSIDE_LEFT_TRENCH(startInsideLeftTrench, "SILT"),
		OUTSIDE_LEFT_TRENCH(startOutsideLeftTrench, "SOLT"),
		LEFT_BUMP(startLeftBump, "SLB"),
		CENTER(startCenter, "SC"),
		RIGHT_BUMP(startRightBump, "SRB"),
		OUTSIDE_RIGHT_TRENCH(startOutsideRightTrench, "SORT"),
		INSIDE_RIGHT_TRENCH(startInsideRightTrench, "SIRT");

		public final AllianceFlipped<Pose2d> pose;
		public final String alias;

		StartingPosition(AllianceFlipped<Pose2d> pose, String alias) {
			this.pose = pose;
			this.alias = alias;
		}
	}
	public enum IntakeLocation {
		FULL_OUTER_SWIPE,
		FULL_INNER_SWIPE,
		HALF_OUTER_SWIPE,
		HALF_INNER_SWIPE,
		OPPONENT_SWIPE,
		DEPOT,
		OUTPOST(true);

		public final boolean requiresReturnPath;

		IntakeLocation(boolean requiresReturnPath) {
			this.requiresReturnPath = requiresReturnPath;
		}

		IntakeLocation() {
			this.requiresReturnPath = false;
		}
	}

	public enum ScoringLocation {
		OUTSIDE_LEFT_TRENCH("OLT"),
		OUTSIDE_RIGHT_TRENCH("ORT"),
		CENTER("Center"),
		LEFT("Left"),
		RIGHT("Right"),
		OUTPOST("O");

		public final String alias;

		ScoringLocation(String alias) {
			this.alias = alias;
		}
	}
}
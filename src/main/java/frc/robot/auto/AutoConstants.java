package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
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

    public static final Distance startLineX = FieldConstants.robotStartingLineCenterX; //NEEDS TO CHANGE
    public static final Distance startXInAllianceZone = startLineX.minus(RobotConstants.centerToFrontBumper); //CORRECT FOR NOW
	public static final Distance startXInTrench = startLineX.minus(RobotConstants.centerToFrontBumper); //NEEDS TO CHANGE FOR INSIDETRENCH!


    public static final AllianceFlipped<Pose2d> FarLeft = 
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            Meters.of(7.399608135223389),
			Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> FarRight = 
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            Meters.of(0.6738224029541016),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> LeftStart =
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            Meters.of(5.887160301208496),
            Rotation2d.k180deg
        ));
        public static final AllianceFlipped<Pose2d> RightStart = 
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            Meters.of(2.307185649871826 ),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> Center = 
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            Meters.of(4.052423000335693),
            Rotation2d.k180deg
        ));
		//The following need to be updated because their starting positions are not the same as preivous ones
    public static final AllianceFlipped<Pose2d> InsideTrenchLeft =
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            FieldConstants.fieldWidth.minus(FieldConstants.Barge.Cage.InnerCage.robotPose.getBlue().getMeasureY()),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> InsideTrenchRight =
        AllianceFlipped.fromBlue(new Pose2d(
            startXInAllianceZone,
            FieldConstants.fieldWidth.div(2),
            Rotation2d.k180deg
        ));
}
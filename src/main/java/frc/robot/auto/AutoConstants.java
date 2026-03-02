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

    public static final Distance startLineX = Inches.of(297.490611); //NEEDS TO CHANGE
    public static final Distance startX = startLineX.minus(RobotConstants.centerToFrontBumper); //CORRECT FOR NOW
    public static final AllianceFlipped<Pose2d> startCenter = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Inches.of(0.0),
			Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startLeftLeftCage = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.Barge.Cage.OuterCage.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startLeftRightCage =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.Barge.Cage.InnerCage.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
        public static final AllianceFlipped<Pose2d> startRightMiddleCage = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.fieldWidth.minus(FieldConstants.Barge.Cage.MiddleCage.robotPose.getBlue().getMeasureY()),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRightRightCage = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.fieldWidth.minus(FieldConstants.Barge.Cage.OuterCage.robotPose.getBlue().getMeasureY()),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRightLeftCage =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.fieldWidth.minus(FieldConstants.Barge.Cage.InnerCage.robotPose.getBlue().getMeasureY()),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startDeadCenter =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            FieldConstants.fieldWidth.div(2),
            Rotation2d.k180deg
        ));
}
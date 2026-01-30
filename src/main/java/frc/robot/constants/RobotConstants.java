package frc.robot.constants;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;

public final class RobotConstants {
	public static final boolean tuningMode = true;

	public static final Mass robotWeight = Pounds.of(125);
	public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(2);

	public static final Distance frameLength = Inches.of(26);
	public static final Distance frameWidth = Inches.of(26);

	public static final Distance centerToFrontFrame = frameLength.div(2);
	public static final Distance centerToSideFrame = frameWidth.div(2);

	public static final Distance bumperWidth = Inches.of(4);

	public static final Distance centerToFrontBumper = centerToFrontFrame.plus(bumperWidth);
	public static final Distance centerToSideBumper = centerToSideFrame.plus(bumperWidth);

	/**Distance between back bumper and front bumper, aka in the X axis */
	public static final Distance robotLength = centerToFrontBumper.times(2);
	/**Distance between left bumper and right bumper, aka in the Y axis */
	public static final Distance robotWidth = centerToSideBumper.times(2);

	public static final Rotation2d intakeForward = Rotation2d.kZero;

	public static final Distance centerToBumperCorner = Meters.of(new Translation2d(robotLength, robotWidth).getNorm());

	public static final double rioUpdatePeriodSecs = Robot.defaultPeriodSecs;
	public static final Time rioUpdatePeriod = Seconds.of(rioUpdatePeriodSecs);
	public static final Frequency rioUpdateFrequency = rioUpdatePeriod.asFrequency();
	public static final double rioUpdateFrequencyHz = rioUpdateFrequency.in(Hertz);

	// public static final double deviceFaultUpdatePeriodSecs = 0.25;
	// public static final Time deviceFaultUpdatePeriod = Seconds.of(deviceFaultUpdatePeriodSecs);
	public static final double deviceFaultUpdateFrequencyHz = 0;
	public static final Frequency deviceFaultUpdateFrequency = Hertz.of(deviceFaultUpdateFrequencyHz);
}

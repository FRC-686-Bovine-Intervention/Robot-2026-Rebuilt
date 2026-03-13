package frc.robot.constants;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;

public final class RobotConstants {
	public static final boolean tuningMode = false;

	public static final Mass robotWeight = Pounds.of(125);
	public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6);

	public static final Distance frameLength = Inches.of(27.0);
	public static final Distance frameWidth = Inches.of(27.0);

	public static final Distance centerToFrontFrame = frameLength.div(2.0);
	public static final Distance centerToSideFrame = frameWidth.div(2.0);

	public static final Distance bumperThickness = Inches.of(3.625);

	public static final Distance centerToFrontBumper = centerToFrontFrame.plus(bumperThickness);
	public static final Distance centerToSideBumper = centerToSideFrame.plus(bumperThickness);

	public static final Translation2d flBumperCorner = new Translation2d(centerToFrontBumper, centerToSideBumper);
	public static final Translation2d frBumperCorner = new Translation2d(centerToFrontBumper, centerToSideBumper.unaryMinus());
	public static final Translation2d blBumperCorner = new Translation2d(centerToFrontBumper.unaryMinus(), centerToSideBumper);
	public static final Translation2d brBumperCorner = new Translation2d(centerToFrontBumper.unaryMinus(), centerToSideBumper.unaryMinus());

	/**Distance between back bumper and front bumper, aka in the X axis */
	public static final Distance robotLength = centerToFrontBumper.times(2.0);
	/**Distance between left bumper and right bumper, aka in the Y axis */
	public static final Distance robotWidth = centerToSideBumper.times(2.0);

	public static final Distance centerToBumperCorner = Meters.of(Math.hypot(centerToFrontBumper.in(Meters), centerToSideBumper.in(Meters)));

	public static final double rioUpdatePeriodSecs = Robot.defaultPeriodSecs;
	public static final Time rioUpdatePeriod = Seconds.of(rioUpdatePeriodSecs);
	public static final Frequency rioUpdateFrequency = rioUpdatePeriod.asFrequency();
	public static final double rioUpdateFrequencyHz = rioUpdateFrequency.in(Hertz);

	// public static final double deviceFaultUpdatePeriodSecs = 0.25;
	// public static final Time deviceFaultUpdatePeriod = Seconds.of(deviceFaultUpdatePeriodSecs);
	public static final double deviceFaultUpdateFrequencyHz = 0.0;
	public static final Frequency deviceFaultUpdateFrequency = Hertz.of(deviceFaultUpdateFrequencyHz);
}

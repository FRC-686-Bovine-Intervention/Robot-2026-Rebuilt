package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
	public static final InterpolatingDoubleTreeMap hubTargetFlyWheelVeloMPS = new InterpolatingDoubleTreeMap();
	static {
		hubTargetFlyWheelVeloMPS.put(Meters.of(1.1717).in(Meters), MetersPerSecond.of(20.0).in(MetersPerSecond));
		hubTargetFlyWheelVeloMPS.put(Meters.of(1.6).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
		hubTargetFlyWheelVeloMPS.put(Meters.of(2.325).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
		hubTargetFlyWheelVeloMPS.put(Meters.of(4.1).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
	}
	public static final InterpolatingDoubleTreeMap hubTargetHoodAngleRads = new InterpolatingDoubleTreeMap();
	static {
		hubTargetHoodAngleRads.put(Meters.of(1.1717).in(Meters), Degrees.of(31.0).in(Radians));
		hubTargetHoodAngleRads.put(Meters.of(1.6).in(Meters), Degrees.of(25.0).in(Radians));
		hubTargetHoodAngleRads.put(Meters.of(2.325).in(Meters), Degrees.of(18.0).in(Radians));
		hubTargetHoodAngleRads.put(Meters.of(4.1).in(Meters), Degrees.of(11.0).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(1.828).in(Meters), Degrees.of(25.0).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(1.907).in(Meters), Degrees.of(22.5).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(2.508).in(Meters), Degrees.of(20.0).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(3.034).in(Meters), Degrees.of(15.0).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(4.131).in(Meters), Degrees.of(12.5).in(Radians));
		// highGoalTargetPivotAltitudeRads.put(Meters.of(4.353).in(Meters), Degrees.of(12.0).in(Radians));
	}



}

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
	public static final InterpolatingDoubleTreeMap hubTargetFlyWheelVeloMPS = new InterpolatingDoubleTreeMap();
	static {
		hubTargetFlyWheelVeloMPS.put(Inches.of(40.125).in(Meters),         MetersPerSecond.of(7.3).in(MetersPerSecond)); // Hub Shot
		hubTargetFlyWheelVeloMPS.put(Inches.of(90.0).in(Meters),          MetersPerSecond.of(7.899966).in(MetersPerSecond));
		hubTargetFlyWheelVeloMPS.put(Inches.of(114.8).in(Meters),          MetersPerSecond.of(8.39996).in(MetersPerSecond));
		hubTargetFlyWheelVeloMPS.put(Inches.of(121.92625).in(Meters),      MetersPerSecond.of(8.39996).in(MetersPerSecond)); // Tower Shot
		hubTargetFlyWheelVeloMPS.put(Inches.of(140.3).in(Meters),          MetersPerSecond.of(8.920215).in(MetersPerSecond)); // Trench Shot
		// hubTargetFlyWheelVeloMPS.put(Inches.of(140.31).in(Meters),          MetersPerSecond.of(9.0).in(MetersPerSecond)); // Trench Shot
		hubTargetFlyWheelVeloMPS.put(Inches.of(183.623178366).in(Meters),  MetersPerSecond.of(9.629733).in(MetersPerSecond)); // Outpost Shot
	}

	public static final InterpolatingDoubleTreeMap hubTargetHoodAngleRads = new InterpolatingDoubleTreeMap();
	static {
		hubTargetHoodAngleRads.put(Inches.of(40.125).in(Meters),        Degrees.of(12.0).in(Radians)); // Hub Shot
		hubTargetHoodAngleRads.put(Inches.of(90.0).in(Meters),         Degrees.of(19.918093).in(Radians));
		hubTargetHoodAngleRads.put(Inches.of(114.8).in(Meters),         Degrees.of(22.026313).in(Radians));
		hubTargetHoodAngleRads.put(Inches.of(121.92625).in(Meters),     Degrees.of(24.962107).in(Radians)); // Tower Shot
		hubTargetHoodAngleRads.put(Inches.of(140.3).in(Meters),         Degrees.of(24.962107).in(Radians)); // Trench Shot
		// hubTargetHoodAngleRads.put(Inches.of(140.31).in(Meters),         Degrees.of(25.153287).in(Radians)); // Trench Shot
		hubTargetHoodAngleRads.put(Inches.of(183.623178366).in(Meters), Degrees.of(29.941957).in(Radians)); // Outpost Shot
	}

	public static final InterpolatingDoubleTreeMap hubTargetTimeOfFlightSecs = new InterpolatingDoubleTreeMap();
	static {
		hubTargetTimeOfFlightSecs.put(Inches.of(40.125).in(Meters),        Seconds.of(0.93).in(Seconds)); // Hub Shot
		hubTargetTimeOfFlightSecs.put(Inches.of(114.8).in(Meters),         Seconds.of(1.0).in(Seconds));
		hubTargetTimeOfFlightSecs.put(Inches.of(121.92625).in(Meters),     Seconds.of(1.2).in(Seconds)); // Tower Shot
		hubTargetTimeOfFlightSecs.put(Inches.of(127.640764646).in(Meters), Seconds.of(0.0).in(Seconds)); // Trench Shot
		hubTargetTimeOfFlightSecs.put(Inches.of(183.623178366).in(Meters), Seconds.of(1.26).in(Seconds)); // Outpost Shot
	}
}

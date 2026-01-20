package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final InterpolatingDoubleTreeMap highGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetFlywheelVeloMPS.put(Meters.of(1.1717).in(Meters), MetersPerSecond.of(20.0).in(MetersPerSecond));
        highGoalTargetFlywheelVeloMPS.put(Meters.of(1.6).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
        highGoalTargetFlywheelVeloMPS.put(Meters.of(2.325).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
        highGoalTargetFlywheelVeloMPS.put(Meters.of(4.1).in(Meters), MetersPerSecond.of(28.0).in(MetersPerSecond));
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetPivotAltitudeRads.put(Meters.of(1.1717).in(Meters), Degrees.of(31.0).in(Radians));
        highGoalTargetPivotAltitudeRads.put(Meters.of(1.6).in(Meters), Degrees.of(25.0).in(Radians));
        highGoalTargetPivotAltitudeRads.put(Meters.of(2.325).in(Meters), Degrees.of(18.0).in(Radians));
        highGoalTargetPivotAltitudeRads.put(Meters.of(4.1).in(Meters), Degrees.of(11.0).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(1.828).in(Meters), Degrees.of(25.0).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(1.907).in(Meters), Degrees.of(22.5).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(2.508).in(Meters), Degrees.of(20.0).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(3.034).in(Meters), Degrees.of(15.0).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(4.131).in(Meters), Degrees.of(12.5).in(Radians));
        // highGoalTargetPivotAltitudeRads.put(Meters.of(4.353).in(Meters), Degrees.of(12.0).in(Radians));
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetDrivetrainOffsetRads.put(Meters.of(1.1717).in(Meters), Degrees.of(0.0).in(Radians));
    }

    public static final InterpolatingDoubleTreeMap lowGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(1.1717).in(Meters), MetersPerSecond.of(4.0).in(MetersPerSecond));
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(1.838).in(Meters), MetersPerSecond.of(6.0).in(MetersPerSecond));
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(1.838).in(Meters), MetersPerSecond.of(6.0).in(MetersPerSecond));
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(3.88).in(Meters), MetersPerSecond.of(7.5).in(MetersPerSecond));
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetPivotAltitudeRads.put(Meters.of(1.415).in(Meters), Degrees.of(15.0).in(Radians));
        lowGoalTargetPivotAltitudeRads.put(Meters.of(1.838).in(Meters), Degrees.of(4.0).in(Radians));
        lowGoalTargetPivotAltitudeRads.put(Meters.of(2.572).in(Meters), Degrees.of(9.0).in(Radians));
        lowGoalTargetPivotAltitudeRads.put(Meters.of(3.88).in(Meters), Degrees.of(7.0).in(Radians));
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetDrivetrainOffsetRads.put(Meters.of(1.1717).in(Meters), Degrees.of(0.0).in(Radians));
    }
}
package frc.util.mechanismUtil;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class LinearRelation {
	private final double effectiveRadiusMeters;

	private LinearRelation(double effectiveRadius) {
		this.effectiveRadiusMeters = effectiveRadius;
	}

	public static LinearRelation wheelRadius(Measure<DistanceUnit> radius) {
		return new LinearRelation(radius.in(Meters));
	}
	public static LinearRelation wheelDiameter(Measure<DistanceUnit> diameter) {
		return wheelRadius(diameter.div(2));
	}
	public static LinearRelation wheelCircumference(Measure<DistanceUnit> circumference) {
		return wheelDiameter(circumference.div(Math.PI));
	}

	public Distance effectiveRadius() {
		return Meters.of(this.effectiveRadiusMeters);
	}

	public double radiansToMeters(double radians) {
		return radians * this.effectiveRadiusMeters;
	}
	public double rotationsToMeters(double rotations){
		return rotations * this.effectiveRadiusMeters * 2 * Math.PI;
	}
	public double metersToRadians(double meters) {
		return meters / this.effectiveRadiusMeters;
	}
	public Distance angleToDistance(Measure<AngleUnit> angle) {
		return Meters.of(this.radiansToMeters(angle.in(Radians)));
	}
	public Angle distanceToAngle(Measure<DistanceUnit> distance) {
		return Radians.of(this.metersToRadians(distance.in(Meters)));
	}
	public LinearVelocity angularVelocityToLinearVelocity(Measure<AngularVelocityUnit> angularVelocity) {
		return MetersPerSecond.of(this.radiansToMeters(angularVelocity.in(RadiansPerSecond)));
	}
	public AngularVelocity linearVelocityToAngularVelocity(Measure<LinearVelocityUnit> linearVelocity) {
		return RadiansPerSecond.of(this.metersToRadians(linearVelocity.in(MetersPerSecond)));
	}
}

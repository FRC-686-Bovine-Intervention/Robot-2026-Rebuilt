package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import frc.util.geometry.GeomUtil;
import frc.util.math.polynomial.TwoVariablePolynomial3rdDegree;

public class ShooterConstants {
	public static final TwoVariablePolynomial3rdDegree hoodPolynomial = TwoVariablePolynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/hoodPolynomial.json");
	public static final TwoVariablePolynomial3rdDegree flywheelPolynomial = TwoVariablePolynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/flywheelPolynomial.json");
	public static final TwoVariablePolynomial3rdDegree tofPolynomial = TwoVariablePolynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/tofPolynomial.json");

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
	}

	// Physics Shooting Calc
	public static final Transform3d flywheelBase = new Transform3d(
		new Translation3d(
			Meters.of(-0.103427),
			Meters.of(0),
			Meters.of(0.492469)
		),
		GeomUtil.rotation3dBuilder()

		.build()
	);
}

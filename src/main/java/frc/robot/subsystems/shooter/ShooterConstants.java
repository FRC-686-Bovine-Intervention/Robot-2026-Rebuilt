package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Filesystem;
import frc.util.math.polynomial.TwoVariablePolynomial3rdDegree;

public class ShooterConstants {
	public static final TwoVariablePolynomial3rdDegree hoodPolynomial = TwoVariablePolynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/hoodPolynomial.json");
	public static final TwoVariablePolynomial3rdDegree flywheelPolynomial = TwoVariablePolynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/flywheelPolynomial.json");
}

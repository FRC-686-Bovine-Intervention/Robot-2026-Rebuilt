package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Filesystem;
import frc.util.math.polynomial.Vector2Polynomial3rdDegree;

public class ShooterConstants {
	public static final Vector2Polynomial3rdDegree aimingPolynomial = Vector2Polynomial3rdDegree.from(Filesystem.getDeployDirectory().getPath() + "/aimingPolynomial.json");
}

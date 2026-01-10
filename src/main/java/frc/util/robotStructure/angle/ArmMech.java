package frc.util.robotStructure.angle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

public class ArmMech extends AngularMech {
	private static final Vector<N3> axis = VecBuilder.fill(0,-1,0);

	public ArmMech(Transform3d base) {
		super(base, axis);
	}
}

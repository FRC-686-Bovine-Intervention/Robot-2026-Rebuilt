package frc.util.robotStructure.linear;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

public class ElevatorMech extends LinearMech {
	private static final Vector<N3> axis = VecBuilder.fill(0,0,1);

	public ElevatorMech(Transform3d base) {
		super(base, axis);
	}
}

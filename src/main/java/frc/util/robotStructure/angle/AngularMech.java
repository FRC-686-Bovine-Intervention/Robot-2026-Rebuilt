package frc.util.robotStructure.angle;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import frc.util.robotStructure.Mechanism3d;

public class AngularMech extends Mechanism3d<AngleUnit> {
	public AngularMech(Transform3d base, Vector<N3> axis) {
		super(base, axis);
	}

	@Override
	public void set(Measure<AngleUnit> angle) {
		this.setRads(angle.in(Radians));
	}

	public void setRads(double angleRads) {
		this.transform = new Transform3d(Translation3d.kZero, new Rotation3d(axis, angleRads));
	}
}

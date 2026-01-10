package frc.util.robotStructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Root implements Component {
	private Pose3d pose = Pose3d.kZero;

	public void setPose(Pose2d pose) {
		this.setPose(new Pose3d(pose));
	}
	public void setPose(Pose3d pose) {
		this.pose = pose;
	}

	@Override
	public Transform3d getRobotRelative() {
		return Transform3d.kZero;
	}

	@Override
	public Pose3d getFieldRelative() {
		return this.pose;
	}
}

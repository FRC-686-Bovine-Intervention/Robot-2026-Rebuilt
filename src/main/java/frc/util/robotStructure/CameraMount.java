package frc.util.robotStructure;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraMount extends ChildBase {
	public static final String KEY = "Camera Overrides";
	private static final List<CameraMount> cameraOverrides = new ArrayList<>();

	public CameraMount(Transform3d base) {
		super(base);
		cameraOverrides.add(this);
	}

	public static void logOverrides() {
		Logger.recordOutput(KEY, cameraOverrides.stream().map(CameraMount::getFieldRelative).toArray(Pose3d[]::new));
	}
}

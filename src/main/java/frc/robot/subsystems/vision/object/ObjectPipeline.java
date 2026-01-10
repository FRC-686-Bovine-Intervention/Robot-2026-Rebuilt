package frc.robot.subsystems.vision.object;

import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraFrame;

public class ObjectPipeline {
	public final Camera camera;
	public final int pipelineIndex;
	public final String name;

	public ObjectPipeline(Camera camera, int pipelineIndex) {
		this(camera, pipelineIndex, camera.toString() + " " + pipelineIndex);
	}
	public ObjectPipeline(Camera camera, int pipelineIndex, String name) {
		this.camera = camera;
		this.pipelineIndex = pipelineIndex;
		this.name = name;
	}

	public CameraFrame[] getFrames() {
		return this.camera.getPipelineFrames(this.pipelineIndex);
	}
}

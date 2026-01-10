package frc.robot.subsystems.vision.apriltag;

import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraFrame;

public class ApriltagPipeline {
	public final Camera camera;
	public final int pipelineIndex;
	public final String name;
	public final double pipelineStdScale;

	public ApriltagPipeline(Camera camera, int pipelineIndex, double pipelineStdScale) {
		this(camera, pipelineIndex, camera.toString() + " " + pipelineIndex, pipelineStdScale);
	}
	public ApriltagPipeline(Camera camera, int pipelineIndex, String name, double pipelineStdScale) {
		this.camera = camera;
		this.pipelineIndex = pipelineIndex;
		this.name = name;
		this.pipelineStdScale = pipelineStdScale;
	}

	public CameraFrame[] getFrames() {
		return this.camera.getPipelineFrames(this.pipelineIndex);
	}
}

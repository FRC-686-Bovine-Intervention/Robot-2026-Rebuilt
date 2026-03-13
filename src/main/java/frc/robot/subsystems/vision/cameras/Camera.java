package frc.robot.subsystems.vision.cameras;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraFrame;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraIOInputs;
import frc.util.LoggedTracer;
import frc.util.robotStructure.CameraMount;

public class Camera extends SubsystemBase {
	private final CameraIO io;
	private final CameraIOInputs inputs;
	private final String name;
	private final Alert disconnectedAlert;
	private final BooleanConsumer connectionCallback;
	public final CameraMount mount;

	public Camera(CameraIO io, String name, Transform3d cameraBase, BooleanConsumer connectionCallback) {
		this.io = io;
		this.inputs = new CameraIOInputs();
		this.name = name;
		this.mount = new CameraMount(cameraBase);
		this.disconnectedAlert = new Alert("Camera \"" + this.name + "\" is not connected", AlertType.kError);
		this.connectionCallback = connectionCallback;

		this.setName("Camera/" + this.name);
	}

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Cameras/" + this.name + "/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Cameras/" + this.name + "/Update Inputs");
		Logger.processInputs("Inputs/Cameras/" + this.name, this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Cameras/" + this.name + "/Process Inputs");

		this.disconnectedAlert.set(!this.inputs.isConnected);
		this.connectionCallback.accept(this.inputs.isConnected);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Cameras/" + this.name + "/Connection Callback");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Cameras/" + this.name);
	}

	public CameraFrame[] getPipelineFrames(int pipelineIndex) {
		return Arrays.stream(this.inputs.frames)
			.filter((frame) -> frame.pipelineIndex == pipelineIndex)
			.toArray(CameraFrame[]::new)
		;
	}

	public Command setPipelineIndex(int pipelineIndex) {
		var subsystem = this;
		return new Command() {
			{
				this.addRequirements(subsystem);
				this.setName("Pipeline " + pipelineIndex);
			}

			@Override
			public void initialize() {
				io.setPipeline(pipelineIndex);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	@Override
	public String toString() {
		return this.name;
	}
}

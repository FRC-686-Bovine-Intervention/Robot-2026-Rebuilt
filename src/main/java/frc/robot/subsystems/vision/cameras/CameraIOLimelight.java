package frc.robot.subsystems.vision.cameras;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.constants.FieldConstants;

public class CameraIOLimelight implements CameraIO {
	private final IntegerSubscriber heartbeatSubsciber;
	// private final DoubleSubscriber latencyPipelineSubscriber;
	// private final DoubleSubscriber latencyCaptureSubscriber;
	private final DoubleArraySubscriber botPoseSubscriber;
	private final DoubleArraySubscriber rawFiducialsSubscriber;

	private long prevHeartbeat = 0;
	private final Debouncer connectionDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	public CameraIOLimelight(String name) {
		final var table = NetworkTableInstance.getDefault().getTable(name);
		this.heartbeatSubsciber = table.getIntegerTopic("hb").subscribe(0);
		this.botPoseSubscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(null, PubSubOption.keepDuplicates(true));
		this.rawFiducialsSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(null, PubSubOption.keepDuplicates(true));
	}

	@Override
	public void updateInputs(CameraIOInputs inputs) {
		var heartbeat = this.heartbeatSubsciber.get();
		inputs.isConnected = this.connectionDebouncer.calculate(heartbeat != this.prevHeartbeat);

		final var botPoses = this.botPoseSubscriber.readQueue();
		final var rawFiducials = this.rawFiducialsSubscriber.readQueueValues();

		final var frameCount = botPoses.length;

		inputs.frames = new CameraFrame[frameCount];

		for (int i = 0; i < frameCount; i++) {
			final var botPose = botPoses[i];

			final var botPoseArray = botPose.value;

			final MultiTagResult multiTagResult;

			final var tagCount = (int) botPoseArray[7];
			final var latencyMS = botPoseArray[6];
			final var cameraTransform = getTransformFromArray(botPose.value);

			if (tagCount > 1) {
				multiTagResult = new MultiTagResult(
					new int[0],
					cameraTransform,
					-1.0,
					cameraTransform,
					-1.0,
					-1.0
				);
			} else {
				multiTagResult = null;
			}

			final CameraTarget[] cameraTargets;

			if (rawFiducials.length > 0) {
				cameraTargets = new CameraTarget[tagCount];
				for (int j = 0; j < tagCount; j++) {
					final var targets = rawFiducials[i];
					final var baseIndex = j * 7;
					final var fiducialID = (int) targets[baseIndex + 0];
					final var ambiguity = targets[baseIndex + 6];

					final var fieldToTag = FieldConstants.apriltagLayout.getTagPose(fiducialID).get();
					final var fieldToCamera = cameraTransform;

					final var cameraToTag = fieldToCamera.inverse().plus(fieldToTag.minus(Pose3d.kZero));

					cameraTargets[j] = new CameraTarget(
						fiducialID,
						-1,
						-7.0,
						-7.0,
						-7.0,
						-7.0,
						cameraToTag,
						cameraToTag,
						ambiguity,
						new Translation2d[0]
					);
				}
			} else {
				cameraTargets = new CameraTarget[0];
			}

			final var ntPublishNS = botPose.timestamp;
			final var timestampSecs = (ntPublishNS / 1e6) - (latencyMS / 1e3);

			inputs.frames[i] = new CameraFrame(
				timestampSecs,
				0,
				cameraTargets,
				Optional.ofNullable(multiTagResult)
			);
		}

		this.prevHeartbeat = heartbeat;
	}

	private static Transform3d getTransformFromArray(double[] array) {
		return new Transform3d(
			new Translation3d(
				array[0],
				array[1],
				array[2]
			),
			new Rotation3d(
				Units.degreesToRadians(array[3]),
				Units.degreesToRadians(array[4]),
				Units.degreesToRadians(array[5])
			)
		);
	}
}

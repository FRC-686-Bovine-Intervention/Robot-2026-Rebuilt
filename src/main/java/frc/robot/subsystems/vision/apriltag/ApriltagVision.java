package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.RobotState.TxTyObservation;
import frc.robot.RobotState.VisionObservation;
import frc.robot.constants.FieldConstants;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class ApriltagVision {
	private final ApriltagPipeline pipelines[];

	private static final LoggedTunable<Distance> zTolerance = LoggedTunable.from("Subsystems/Vision/Apriltags/Filtering/Z Tolerance", Inches::of, 18.0);
	private static final LoggedTunable<Angle> gyroTolerance = LoggedTunable.from("Subsystems/Vision/Apriltags/Filtering/Gyro Tolerance", Degrees::of, 10.0);
	private static final LoggedTunableNumber xyStdDevCoef = LoggedTunable.from("Subsystems/Vision/Apriltags/Std Devs/XY Coef", 0.01);
	private static final LoggedTunableNumber thetaStdDevCoef = LoggedTunable.from("Subsystems/Vision/Apriltags/Std Devs/Theta Coef", Double.POSITIVE_INFINITY);

	public ApriltagVision(ApriltagPipeline... pipelines) {
		System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
		this.pipelines = pipelines;
	}

	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/ApriltagVision/Before");
		final List<VisionObservation> allVisionObservations = new ArrayList<>(this.pipelines.length * 3);
		final Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>(this.pipelines.length * 3);

		for (final var pipeline : this.pipelines) {
			final var frames = pipeline.getFrames();
			final var loggingKey = "Subsystems/Vision/Apriltags/Results/" + pipeline.name;
			final var tracingKey = "CommandScheduler Periodic/ApriltagVision/Process Results/" + pipeline.name;

			var akitPose3d = new Pose3d[0];
			var akitTargetCorners = new Translation2d[0];

			for (final var frame : frames) {
				final var usableTags = new ArrayList<AprilTag>(frame.targets.length);

				for (final var target : frame.targets) {
					final var tagID = target.tagID;
					final var optTagPose = FieldConstants.apriltagLayout.getTagPose(tagID);
					if (optTagPose.isEmpty()) {
						continue;
					}
					final var tagPose = optTagPose.get();

					usableTags.add(new AprilTag(tagID, tagPose));

					final var previousObservation = allTxTyObservations.get(tagID);
					if (previousObservation == null || frame.timestamp > previousObservation.timestamp()) {
						final var translationToTarget = target.bestCameraToTag.getTranslation();
						final var cameraRotation = pipeline.camera.mount.getFieldRelative().getRotation();
						final var tagRotationRelativeToCamera = tagPose.getRotation().minus(cameraRotation);
						final var cameraToTag = new Transform3d(translationToTarget, tagRotationRelativeToCamera);
						final var cameraPose = tagPose.transformBy(cameraToTag.inverse());
						final var robotPose = cameraPose.transformBy(pipeline.camera.mount.getRobotRelative().inverse());
						final var correctedObservationTimestamp = Math.min(frame.timestamp, Timer.getTimestamp());

						allTxTyObservations.put(
							tagID,
							new TxTyObservation(
								correctedObservationTimestamp,
								tagID,
								robotPose.toPose2d()
							)
						);
					}
				}

				Logger.recordOutput(loggingKey + "/Targets/Tag IDs", usableTags.stream().mapToInt((tag) -> tag.ID).toArray());
				Logger.recordOutput(loggingKey + "/Targets/Tag Poses", usableTags.stream().map((tag) -> tag.pose).toArray(Pose3d[]::new));
				akitTargetCorners = Arrays.stream(frame.targets).flatMap((target) -> Arrays.stream(target.corners)).toArray(Translation2d[]::new);

				if (usableTags.isEmpty()) {
					continue;
				}

				// final because averageTagDist mapToDouble needs it
				final Pose3d cameraPose3d;
				final Pose3d robotPose3d;
				var useVisionRotation = false;

				if (frame.multiTagResult.isPresent()) {
					final var multiTagResult = frame.multiTagResult.get();
					cameraPose3d = new Pose3d(
						multiTagResult.bestTransform.getTranslation(),
						multiTagResult.bestTransform.getRotation()
					);
					robotPose3d = cameraPose3d.transformBy(pipeline.camera.mount.getRobotRelative().inverse());
					useVisionRotation = true;
				} else if (usableTags.size() == 1) {
					final var target = frame.targets[0];
					final var tagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get();
					final var translationToTarget = target.bestCameraToTag.getTranslation();
					final var cameraRotation = pipeline.camera.mount.getFieldRelative().getRotation();
					final var tagRotationRelativeToCamera = tagPose.getRotation().minus(cameraRotation);
					final var cameraToTag = new Transform3d(translationToTarget, tagRotationRelativeToCamera);
					final var cameraPose = tagPose.transformBy(cameraToTag.inverse());
					final var robotPose = cameraPose.transformBy(pipeline.camera.mount.getRobotRelative().inverse());

					cameraPose3d = cameraPose;
					robotPose3d = robotPose;
					useVisionRotation = false;
					// var bestCameraPose = tagPose.transformBy(target.bestCameraToTag.inverse());
					// var bestRobotPose = bestCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
					// var altCameraPose = tagPose.transformBy(target.altCameraToTag.inverse());
					// var altRobotPose = altCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
					// if (frame.targets[0].poseAmbiguity < ambiguityThreshold.get()) {
					//     var currentRotation = RobotState.getInstance().getPose().getRotation();
					//     var bestRotation = bestRobotPose.getRotation().toRotation2d();
					//     var altRotation = altRobotPose.getRotation().toRotation2d();
					//     if (Math.abs(currentRotation.minus(bestRotation).getRadians()) < Math.abs(currentRotation.minus(altRotation).getRadians())) {
					//         cameraPose3d = bestCameraPose;
					//         robotPose3d = bestRobotPose;
					//     } else {
					//         cameraPose3d = altCameraPose;
					//         robotPose3d = altRobotPose;
					//     }
					// } else {
					//     cameraPose3d = null;
					//     robotPose3d = null;
					// }

				} else {
					Logger.recordOutput(loggingKey + "/Robot pose null", true);
					Logger.recordOutput(loggingKey + "/Camera pose null", true);
					continue;
				}
				Logger.recordOutput(loggingKey + "/Robot pose null", false);
				Logger.recordOutput(loggingKey + "/Camera pose null", false);
				
				final var robotPose2d = robotPose3d.toPose2d();
				akitPose3d = new Pose3d[]{robotPose3d};
				Logger.recordOutput(loggingKey + "/Poses/Robot2d", robotPose2d);
				// Logger.recordOutput(loggingKey + "/Poses/Robot3d", robotPose3d);
				Logger.recordOutput(loggingKey + "/Poses/Camera3d", cameraPose3d);

				// Filtering
				final var inField = ApriltagVisionConstants.acceptableFieldBox.withinBounds(robotPose2d.getTranslation());
				final var closeToFloor = MathUtil.isNear(0.0, robotPose3d.getZ(), ApriltagVision.zTolerance.get().in(Meters));
				final var closeToGyro = robotPose2d.getRotation().minus(RobotState.getInstance().getEstimatedGlobalPose().getRotation()).getCos() > Math.cos(gyroTolerance.get().in(Radians));
				final var gyroFilter = closeToGyro || usableTags.size() >= 2;

				Logger.recordOutput(loggingKey + "/Filtering/In Field", inField);
				Logger.recordOutput(loggingKey + "/Filtering/Close to Floor", closeToFloor);
				Logger.recordOutput(loggingKey + "/Filtering/Close to Gyro", gyroFilter);

				if (
					!inField
					|| !closeToFloor
					|| !gyroFilter
				) {
					continue;
				}

				// Std Devs
				var totalTagDistance = 0.0;
				for (final var tag : usableTags) {
					totalTagDistance += tag.pose.getTranslation().getDistance(cameraPose3d.getTranslation());
				}
				final var averageTagDistance = totalTagDistance / usableTags.size();
				Logger.recordOutput(loggingKey + "/Std Devs/Average Distance", averageTagDistance);

				final var xyStdDev =
					xyStdDevCoef.get()
					* Math.pow(averageTagDistance, 1.2)
					/ Math.pow(usableTags.size(), 2.0)
					* pipeline.pipelineStdScale
				;
				final var thetaStdDev =
					(useVisionRotation) ? (
						thetaStdDevCoef.get()
						* Math.pow(averageTagDistance, 1.2)
						/ Math.pow(usableTags.size(), 2.0)
						* pipeline.pipelineStdScale
					) : (
						Double.POSITIVE_INFINITY
					)
				;
				Logger.recordOutput(loggingKey + "/Std Devs/XY", xyStdDev);
				Logger.recordOutput(loggingKey + "/Std Devs/Theta", thetaStdDev);

				allVisionObservations.add(new VisionObservation(
					frame.timestamp,
					robotPose2d,
					VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
				));
			}
			Logger.recordOutput(loggingKey + "/Poses/Robot3d", akitPose3d);
			Logger.recordOutput(loggingKey + "/Targets/Target Corners", akitTargetCorners);
			Logger.recordOutput(loggingKey + "/Frame Count", frames.length);

			LoggedTracer.logEpoch(tracingKey);
		}

		LoggedTracer.logEpoch("CommandScheduler Periodic/ApriltagVision/Process Results");

		allVisionObservations.stream()
			.sorted(Comparator.comparingDouble(VisionObservation::timestamp))
			.forEachOrdered(RobotState.getInstance()::addVisionObservation)
		;
		allTxTyObservations.values().stream()
			.forEachOrdered(RobotState.getInstance()::addTxTyObservation)
		;

		LoggedTracer.logEpoch("CommandScheduler Periodic/ApriltagVision/Send Observations");
		LoggedTracer.logEpoch("CommandScheduler Periodic/ApriltagVision");
	}
}

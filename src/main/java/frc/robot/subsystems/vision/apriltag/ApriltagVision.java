package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
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

	private static final LoggedTunable<Angle> gyroTolerance = LoggedTunable.from("Vision/Apriltags/Filtering/Gyro Tolerance", Degrees::of, 10);
	private static final LoggedTunableNumber xyStdDevCoef = LoggedTunable.from("Vision/Apriltags/Std Devs/XY Coef", 0.01);
	private static final LoggedTunableNumber thetaStdDevCoef = LoggedTunable.from("Vision/Apriltags/Std Devs/Theta Coef", Double.POSITIVE_INFINITY);

	public ApriltagVision(ApriltagPipeline... pipelines) {
		System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
		this.pipelines = pipelines;
	}

	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/ApriltagVision/Before");
		List<VisionObservation> allVisionObservations = new ArrayList<>(this.pipelines.length * 3);
		Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>(this.pipelines.length * 3);

		for (var pipeline : this.pipelines) {
			var frames = pipeline.getFrames();
			var loggingKey = "Vision/Apriltags/Results/" + pipeline.name;
			var tracingKey = "CommandScheduler Periodic/ApriltagVision/Process Results/" + pipeline.name;
			var akitPose3d = new Pose3d[0];
			var akitTargetCorners = new Translation2d[0];
			for (var frame : frames) {
				var usableTags = Arrays
					.stream(frame.targets)
					.map((target) -> {
						var optTagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID);
						if (optTagPose.isEmpty()) return Optional.empty();
						return Optional.of(new AprilTag(target.tagID, optTagPose.get()));
					})
					.filter(Optional::isPresent)
					.map(Optional::get)
					.toArray(AprilTag[]::new)
				;
				Logger.recordOutput(loggingKey + "/Targets/Tag IDs", Arrays.stream(usableTags).mapToInt((tag) -> tag.ID).toArray());
				Logger.recordOutput(loggingKey + "/Targets/Tag Poses", Arrays.stream(usableTags).map((tag) -> tag.pose).toArray(Pose3d[]::new));
				akitTargetCorners = Arrays.stream(frame.targets).flatMap((target) -> Arrays.stream(target.corners)).toArray(Translation2d[]::new);

				if (frame.targets.length == 0) {continue;}

				for (var target : frame.targets) {
					var tagID = target.tagID;
					if (tagID == -1) {continue;}
					var previousObservation = allTxTyObservations.get(tagID);
					if (previousObservation == null || frame.timestamp > previousObservation.timestamp()) {
						var tagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get();
						var translationToTarget = target.bestCameraToTag.getTranslation();
						var cameraRotation = pipeline.camera.mount.getFieldRelative().getRotation();
						var tagRotationRelativeToCamera = tagPose.getRotation().minus(cameraRotation);
						var cameraToTag = new Transform3d(translationToTarget, tagRotationRelativeToCamera);
						var cameraPose = tagPose.transformBy(cameraToTag.inverse());
						var robotPose = cameraPose.transformBy(pipeline.camera.mount.getRobotRelative().inverse());
						var correctedObservationTimestamp = Math.min(frame.timestamp, Timer.getTimestamp());

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

				// final because averageTagDist mapToDouble needs it
				final Pose3d cameraPose3d;
				final Pose3d robotPose3d;
				var useVisionRotation = false;

				if (frame.multiTagResult.isPresent()) {
					var multiTagResult = frame.multiTagResult.get();
					cameraPose3d = new Pose3d(
						multiTagResult.bestTransform.getTranslation(),
						multiTagResult.bestTransform.getRotation()
					);
					robotPose3d = cameraPose3d.transformBy(pipeline.camera.mount.getRobotRelative().inverse());
					useVisionRotation = true;
				} else if (frame.targets.length == 1) {
					var target = frame.targets[0];
					var tagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get();
					var translationToTarget = target.bestCameraToTag.getTranslation();
					var cameraRotation = pipeline.camera.mount.getFieldRelative().getRotation();
					var tagRotationRelativeToCamera = tagPose.getRotation().minus(cameraRotation);
					var cameraToTag = new Transform3d(translationToTarget, tagRotationRelativeToCamera);
					var cameraPose = tagPose.transformBy(cameraToTag.inverse());
					var robotPose = cameraPose.transformBy(pipeline.camera.mount.getRobotRelative().inverse());

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
					cameraPose3d = null;
					robotPose3d = null;
				}
				if (robotPose3d == null || cameraPose3d == null) {
					Logger.recordOutput(loggingKey + "/Robot pose null", robotPose3d == null);
					Logger.recordOutput(loggingKey + "/Camera pose null", cameraPose3d == null);
					continue;
				}
				Logger.recordOutput(loggingKey + "/Robot pose null", false);
				Logger.recordOutput(loggingKey + "/Camera pose null", false);
				var robotPose2d = robotPose3d.toPose2d();
				akitPose3d = new Pose3d[]{robotPose3d};
				Logger.recordOutput(loggingKey + "/Poses/Robot2d", robotPose2d);
				// Logger.recordOutput(loggingKey + "/Poses/Robot3d", robotPose3d);
				Logger.recordOutput(loggingKey + "/Poses/Camera3d", cameraPose3d);

				// Filtering
				var inField = ApriltagVisionConstants.acceptableFieldBox.withinBounds(robotPose2d.getTranslation());
				var closeToFloor = robotPose3d.getTranslation().getMeasureZ().isNear(Meters.zero(), ApriltagVisionConstants.zMargin);
				var closeToGyro = robotPose2d.getRotation().minus(RobotState.getInstance().getEstimatedGlobalPose().getRotation()).getCos() > Math.cos(gyroTolerance.get().in(Radians));
				var gyroFilter = closeToGyro || usableTags.length >= 2;

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
				var optAverageTagDistance = Arrays
					.stream(usableTags)
					.mapToDouble((tag) -> tag.pose.getTranslation().getDistance(cameraPose3d.getTranslation()))
					.average()
				;
				if (optAverageTagDistance.isEmpty()) continue;
				var averageTagDistance = optAverageTagDistance.getAsDouble();
				Logger.recordOutput(loggingKey + "/Std Devs/Average Distance", averageTagDistance);

				double xyStdDev =
					xyStdDevCoef.get()
					* Math.pow(averageTagDistance, 1.2)
					/ Math.pow(usableTags.length, 2.0)
					* pipeline.pipelineStdScale
				;
				double thetaStdDev =
					(useVisionRotation) ? (
						thetaStdDevCoef.get()
						* Math.pow(averageTagDistance, 1.2)
						/ Math.pow(usableTags.length, 2.0)
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

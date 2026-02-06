package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraTarget;
import frc.robot.subsystems.vision.object.ObjectVisionConstants.TrackableObject;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.CameraMount;

public class ObjectVision {
	private final ObjectPipeline[] pipelines;

	private static final LoggedTunable<Distance> updateDistanceThreshold = LoggedTunable.from("Vision/Object/Updating/Update Distance Threshold", Meters::of, 0.1);
	private static final LoggedTunableNumber posUpdatingFilteringFactor =  LoggedTunable.from("Vision/Object/Updating/Pos Updating Filtering Factor", 0.8);
	//private static final LoggedTunableNumber confUpdatingFilteringFactor = LoggedTunable.from("Vision/Object/Confidence/Updating Filtering Factor", 0.5);
	private static final LoggedTunableNumber confidenceDecayPerSecond =    LoggedTunable.from("Vision/Object/Confidence/Decay Per Second", 3);
	private static final LoggedTunableNumber detargetConfidenceThreshold = LoggedTunable.from("Vision/Object/Target Threshold/Detarget", 0.5);

	private List<List<TrackedObject>> allTrackedObjects = new ArrayList<>(0);

	public ObjectVision(ObjectPipeline... pipelines) {
		System.out.println("[Init ObjectVision] Instantiating ObjectVision");
		this.pipelines = pipelines;

		for (int i = 0; i < ObjectVisionConstants.trackableObjects.length; i++) {
			allTrackedObjects.add(new ArrayList<>());
		}
	}

	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/ObjectVision/Before");
		for (var pipeline : this.pipelines) {
			var frames = pipeline.getFrames();
			var loggingKey = "Vision/Objects/Results/" + pipeline.pipelineIndex + "/";
			for (var frame : frames) {
				List<List<TrackedObject>> allFrameTargets = new ArrayList<>();
				for (int i = 0; i < ObjectVisionConstants.trackableObjects.length; i++) {
					allFrameTargets.add(new ArrayList<>());
				}

				for (int i = 0; i < frame.targets.length; i++) {
					var target = TrackedObject.from(pipeline.camera.mount, frame.targets[i]);
					if (target.isPresent()) {
						allFrameTargets.get(target.get().type.classId).add(target.get());
					}
				}


				for (var trackedObjects : allTrackedObjects) {
					for (var trackedObject : trackedObjects) {
						trackedObject.resetUpdated();
						trackedObject.decayConfidence(confidenceDecayPerSecond.getAsDouble());
					}
				}
				for (int i = 0; i < allFrameTargets.size(); i++) {
					var frameTargets = allFrameTargets.get(i);
					var trackedObjects = allTrackedObjects.get(i);
					for (int j = 0; j < frameTargets.size(); j++) {
						var frameTarget = frameTargets.get(j);
						for (var trackedObject : trackedObjects) {
							if (trackedObject.fieldPos.getDistance(frameTarget.fieldPos) < updateDistanceThreshold.get().in(Meters) && !trackedObject.updated) {
								trackedObject.updateWithFiltering(frameTarget);
							} else {
								trackedObjects.add(frameTarget);
							}
						}
					}
				}

				List<List<Pose3d>> allLoggedTargets = new ArrayList<>();
				for (int i = 0; i < ObjectVisionConstants.trackableObjects.length; i++) {
					allLoggedTargets.add(new ArrayList<>());
				}
				for (int i = 0; i < allTrackedObjects.size(); i++) {
					var trackedObjects = allTrackedObjects.get(i);
					var loggedTargets = allLoggedTargets.get(i);
					
					for (var trackedObject : trackedObjects) {
						if (trackedObject.confidence < detargetConfidenceThreshold.getAsDouble()) {
							trackedObjects.remove(trackedObject);
						}

						loggedTargets.add(trackedObject.toASPose());
					}
				}
				for (int i = 0; i < ObjectVisionConstants.trackableObjects.length; i++) {
					Logger.recordOutput(loggingKey + "Targets/" + ObjectVisionConstants.trackableObjects[i].className, (Pose3d[]) allLoggedTargets.get(i).toArray());
				}
			}
		}
	}

	public static class TrackedObject {
		public TrackableObject type;
		public Translation2d fieldPos;
		public double confidence;
		public boolean updated;

		public TrackedObject(TrackableObject type, Translation2d fieldPos, double confidence) {
			this.type = type;
			this.fieldPos = fieldPos;
			this.confidence = confidence;
			this.updated = true;
		}

		public static Optional<TrackedObject> from(CameraMount mount, CameraTarget target) {
			var camRobotPose = mount.getRobotRelative();
			var camFieldPose = mount.getFieldRelative();

			var tty = target.pitchRads + camRobotPose.getRotation().getY();
			var ttx = -target.yawRads;

			var type = ObjectVisionConstants.trackableObjects[target.objectClassID];

			var height = type.height.in(Meters) / 2.0 - camRobotPose.getTranslation().getZ();
			var h = height / Math.tan(tty);
			var x = Math.cos(ttx) * h;
			var y = Math.sin(ttx) * h;

			var objectFieldPose = camFieldPose.toPose2d().transformBy(new Transform2d(
				new Translation2d(
					x,
					y
				),
				Rotation2d.kZero
			));

			return Optional.of(new TrackedObject(type, objectFieldPose.getTranslation(), target.objectConfidence));
		}

		// public void updateConfidence() {
		//     confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1);
		// }

		public void updateWithFiltering(TrackedObject newObject) {
			this.fieldPos = fieldPos.interpolate(newObject.fieldPos, posUpdatingFilteringFactor.get());
			this.confidence = newObject.confidence;
			this.updated = true;
		}

		public void resetUpdated() {
			this.updated = false;
		}

		public void decayConfidence(double rate) {
			this.confidence -= confidenceDecayPerSecond.get() * rate * RobotConstants.rioUpdatePeriodSecs;
		}

		// public double getPriority(ChassisSpeeds desiredRobotRelativeSpeeds) {
		//     // var pose = RobotState.getInstance().getEstimatedGlobalPose();
		//     // var FORR = fieldPos.minus(pose.getTranslation());
		//     // var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
		//     // return
		//     //     confidence * priorityPerConfidence.get() *
		//     //     VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) +
		//     //     FORR.getNorm() * priorityPerDistance.get()
		//     // ;
		//     var intakePose = RobotState.getInstance().getEstimatedGlobalPose(); //TODO: Replace with a mount where the intake would be
		//     var rel = new Pose2d(this.fieldPos, Rotation2d.kZero).relativeTo(intakePose);

		//     double dist = Math.hypot(rel.getX(), 2 * rel.getY());
		//     double angle = Math.atan2(rel.getY(), rel.getX());

		//     double score = 0;

		//     var chassisSpeedsAngle = Math.atan2(desiredRobotRelativeSpeeds.vxMetersPerSecond, desiredRobotRelativeSpeeds.vyMetersPerSecond);
		//     if (Math.abs(angle - chassisSpeedsAngle) < Math.PI / 2 && (angle > Math.PI/2 || angle < -Math.PI/2)) {
		//         score = (1 / (dist * priorityPerDistance.getAsDouble())) * (this.confidence * priorityPerConfidence.getAsDouble());
		//     }

		//     return score;
		// }

		public Pose3d toASPose() {
			return new Pose3d(
				new Translation3d(
					fieldPos.getMeasureX(),
					fieldPos.getMeasureY(),
					type.height.div(2)
				),
				new Rotation3d(
					Degrees.of(180),
					Degrees.zero(),
					Degrees.zero()
				)
			);
		}

		// public static final TrackedObjectStruct struct = new TrackedObjectStruct();
		// public static class TrackedObjectStruct implements Struct<TrackedObject> {
		//     @Override
		//     public Class<TrackedObject> getTypeClass() {
		//         return TrackedObject.class;
		//     }

		//     @Override
		//     public String getTypeName() {
		//         return "TrackedNote";
		//     }

		//     @Override
		//     public int getSize() {
		//         return Translation2d.struct.getSize() * 1 + kSizeDouble * 1;
		//     }

		//     @Override
		//     public String getSchema() {
		//         return "Translation2d fieldPos;double confidence";
		//     }

		//     @Override
		//     public Struct<?>[] getNested() {
		//         return new Struct<?>[] {Translation2d.struct};
		//     }

		//     @Override
		//     public TrackedObject unpack(ByteBuffer bb) {
		//         var type = bb.getInt();
		//         var fieldPos = Translation2d.struct.unpack(bb);
		//         var confidence = bb.getDouble();
		//         return new TrackedObject(type, fieldPos, confidence);
		//     }

		//     @Override
		//     public void pack(ByteBuffer bb, TrackedObject value) {
		//         Translation2d.struct.pack(bb, value.fieldPos);
		//         bb.putDouble(value.confidence);
		//     }
		// }
	}
}

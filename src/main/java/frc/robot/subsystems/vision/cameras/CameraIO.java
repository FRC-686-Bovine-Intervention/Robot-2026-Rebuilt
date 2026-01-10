package frc.robot.subsystems.vision.cameras;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.util.loggerUtil.LoggerUtil;

public interface CameraIO {

	public default void updateInputs(CameraIOInputs inputs) {}

	public default void setPipeline(int pipelineIndex) {}

	public default void setDriverMode(boolean driverMode) {}

	// TODO
	public default void setLEDMode() {}

	public static class CameraIOInputs implements LoggableInputs {
		public boolean isConnected = false;
		public CameraFrame[] frames = new CameraFrame[0];

		@Override
		public void toLog(LogTable table) {
			table.put("IsConnected", this.isConnected);
			LoggerUtil.toLogArray(table.getSubtable("Frames"), this.frames);
		}

		@Override
		public void fromLog(LogTable table) {
			this.isConnected = table.get("IsConnected", this.isConnected);
			this.frames = LoggerUtil.fromLogArray(table.getSubtable("Frames"), CameraFrame::new, CameraFrame[]::new);
		}
	}

	public static class CameraFrame implements LoggableInputs {
		public double timestamp;
		public int pipelineIndex;
		public CameraTarget[] targets;
		public Optional<MultiTagResult> multiTagResult;

		public CameraFrame() {
			this(
				0.0,
				0,
				new CameraTarget[0],
				Optional.empty()
			);
		}

		public CameraFrame(double timestamp, int pipelineIndex, CameraTarget[] targets, Optional<MultiTagResult> multiTagResult) {
			this.timestamp = timestamp;
			this.pipelineIndex = pipelineIndex;
			this.targets = targets;
			this.multiTagResult = multiTagResult;
		}

		@Override
		public void toLog(LogTable table) {
			table.put("Timestamp", this.timestamp);
			table.put("PipelineIndex", this.pipelineIndex);
			LoggerUtil.toLogArray(table.getSubtable("Targets"), this.targets);
			LoggerUtil.toLogOptional(table.getSubtable("MultiTagResult"), this.multiTagResult);
		}

		@Override
		public void fromLog(LogTable table) {
			this.timestamp = table.get("Timestamp", this.timestamp);
			this.pipelineIndex = table.get("PipelineIndex", this.pipelineIndex);
			this.targets = LoggerUtil.fromLogArray(table.getSubtable("Targets"), CameraTarget::new, CameraTarget[]::new);
			this.multiTagResult = LoggerUtil.fromLogOptional(table.getSubtable("MultiTagResult"), MultiTagResult::new);
		}
	}

	public static class CameraTarget implements LoggableInputs {
		public int tagID;
		public int objectClassID;
		public double objectConfidence;
		public double yawRads;
		public double pitchRads;
		public double skewRads;
		public Transform3d bestCameraToTag;
		public Transform3d altCameraToTag;
		public double poseAmbiguity;
		public Translation2d[] corners;

		public CameraTarget() {
			this(
				-1,
				-1,
				0.0,
				0.0,
				0.0,
				0.0,
				Transform3d.kZero,
				Transform3d.kZero,
				0.0,
				new Translation2d[0]
			);
		}

		public CameraTarget(
			int tagID,
			int objectClassID,
			double objectConfidence,
			double yawRads,
			double pitchRads,
			double skewRads,
			Transform3d bestCameraToTag,
			Transform3d altCameraToTag,
			double poseAmbiguity,
			Translation2d[] corners
		) {
			this.tagID = tagID;
			this.objectClassID = objectClassID;
			this.objectConfidence = objectConfidence;
			this.yawRads = yawRads;
			this.pitchRads = pitchRads;
			this.skewRads = skewRads;
			this.bestCameraToTag = bestCameraToTag;
			this.altCameraToTag = altCameraToTag;
			this.poseAmbiguity = poseAmbiguity;
			this.corners = corners;
		}

		public CameraTarget(
			int tagID,
			double yawRads,
			double pitchRads,
			double skewRads,
			Transform3d bestCameraToTag,
			Transform3d altCameraToTag,
			double poseAmbiguity,
			Translation2d[] corners
		) {
			this.tagID = tagID;
			this.yawRads = yawRads;
			this.pitchRads = pitchRads;
			this.skewRads = skewRads;
			this.bestCameraToTag = bestCameraToTag;
			this.altCameraToTag = altCameraToTag;
			this.poseAmbiguity = poseAmbiguity;
			this.corners = corners;
		}

		@Override
		public void toLog(LogTable table) {
			table.put("TagID", this.tagID);
			table.put("ObjectClassID", this.objectClassID);
			table.put("ObjectConfidence", this.objectConfidence);
			table.put("YawRads", this.yawRads);
			table.put("PitchRads", this.pitchRads);
			table.put("SkewRads", this.skewRads);
			table.put("BestCameraToTag", this.bestCameraToTag);
			table.put("AltCameraToTag", this.altCameraToTag);
			table.put("PoseAmbiguity", this.poseAmbiguity);
			table.put("Corners", this.corners);
		}
		@Override
		public void fromLog(LogTable table) {
			this.tagID = table.get("TagID", this.tagID);
			this.objectClassID = table.get("ObjectClassID", this.objectClassID);
			this.objectConfidence = table.get("ObjectConfidence", this.objectConfidence);
			this.yawRads = table.get("YawRads", this.yawRads);
			this.pitchRads = table.get("PitchRads", this.pitchRads);
			this.skewRads = table.get("SkewRads", this.skewRads);
			this.bestCameraToTag = table.get("BestCameraToTag", this.bestCameraToTag);
			this.altCameraToTag = table.get("AltCameraToTag", this.altCameraToTag);
			this.poseAmbiguity = table.get("PoseAmbiguity", this.poseAmbiguity);
			this.corners = table.get("Corners", this.corners);
		}
	}

	public static class MultiTagResult implements LoggableInputs {
		public int[] tagIDs;
		public Transform3d bestTransform;
		public double bestReprojectionError;
		public Transform3d altTransform;
		public double altReprojectionError;
		public double ambiguity;

		public MultiTagResult() {
			this(
				new int[0],
				Transform3d.kZero,
				0.0,
				Transform3d.kZero,
				0.0,
				0.0
			);
		}

		public MultiTagResult(int[] tagIDs, Transform3d bestTransform, double bestReprojectionError, Transform3d altTransform, double altReprojectionError, double ambiguity) {
			this.tagIDs = tagIDs;
			this.bestTransform = bestTransform;
			this.bestReprojectionError = bestReprojectionError;
			this.altTransform = altTransform;
			this.altReprojectionError = altReprojectionError;
			this.ambiguity = ambiguity;
		}

		@Override
		public void toLog(LogTable table) {
			table.put("TagIDs", this.tagIDs);
			table.put("BestTransform", this.bestTransform);
			table.put("BestReprojectionError", this.bestReprojectionError);
			table.put("AltTransform", this.altTransform);
			table.put("AltReprojectionError", this.altReprojectionError);
			table.put("Ambiguity", this.ambiguity);
		}
		@Override
		public void fromLog(LogTable table) {
			this.tagIDs = table.get("TagIDs", this.tagIDs);
			this.bestTransform = table.get("BestTransform", this.bestTransform);
			this.bestReprojectionError = table.get("BestReprojectionError", this.bestReprojectionError);
			this.altTransform = table.get("AltTransform", this.altTransform);
			this.altReprojectionError = table.get("AltReprojectionError", this.altReprojectionError);
			this.ambiguity = table.get("Ambiguity", this.ambiguity);
		}
	}
}

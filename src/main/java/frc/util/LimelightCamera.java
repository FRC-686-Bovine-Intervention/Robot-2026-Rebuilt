package frc.util;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import lombok.Getter;

public class LimelightCamera {
	private final String name;
	private final DoubleSubscriber heartbeatSubscriber;
	private final StringSubscriber pipelineTypeSubscriber;
	private final DoubleArraySubscriber rawFiducialSubscriber;
	private final DoubleArraySubscriber rawDetectionSubscriber;


	public LimelightCamera(String name) {
		this.name = name;

		var inputTable = NetworkTableInstance.getDefault().getTable(name);

		this.heartbeatSubscriber = inputTable
			.getDoubleTopic("hb")
			.subscribe(0);
		
		this.pipelineTypeSubscriber = inputTable
			.getStringTopic("getpipetype")
			.subscribe("apriltag");
		
		this.rawFiducialSubscriber = inputTable
			.getDoubleArrayTopic("rawfiducials")
			.subscribe(new double[0]);

		this.rawDetectionSubscriber = inputTable
			.getDoubleArrayTopic("rawdetections")
			.subscribe(new double[0]);
	}


	public double getHeartbeat() {
		return this.heartbeatSubscriber.getAsDouble();
	}

	public LimelightResult getResults() {

	}

	private LimelightTarget[] getTargets() {
		if (pipelineTypeSubscriber.get() == "apriltag") {
			//rawfiducials
			var rawFiducialArray = rawFiducialSubscriber.get();
			int valsPerEntry = 7;

			if (rawFiducialArray.length % valsPerEntry != 0) {
				return new LimelightTarget[0];
			}

			int numFiducials = rawFiducialArray.length / valsPerEntry;
			LimelightTarget[] rawFiducials = new LimelightTarget[numFiducials];

			for (int i = 0; i < numFiducials; i++) {
				int baseIndex = i * valsPerEntry;
				int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
				double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
				double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            	double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            	double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            	double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            	double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);
            
            	rawFiducials[i] = new LimelightTarget(
					id,
					-1,
					-1,
					txnc,
					tync,
					0.0,
					null, //NEEDS SOLVED
					null,  //NEEDS SOLVED
					ambiguity,
					null //NEEDS SOLVED
				);
			}
		} else {
			
		}
	}

	private static double extractArrayEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

	public static class LimelightResult {
		@Getter
		private final double timestampSeconds;
		@Getter
		private final LimelightTarget[] frameTargets;

		private LimelightResult(double timestampSeconds, LimelightTarget[] frameTargets) {
			this.timestampSeconds = timestampSeconds;
			this.frameTargets = frameTargets;
		}
	}

	public static class LimelightTarget {
		@Getter
		private final int fiducialID;
		@Getter
		private final int objectClassID;
		@Getter
		private final double detectedObjectConfidence;
		@Getter
		private final double yawRads;
		@Getter
		private final double pitchRads;
		@Getter
		private final double skewRads;
		@Getter
		private final Transform3d bestCameraToTag;
		@Getter
		private final Transform3d altCameraToTag;
		@Getter
		private final double poseAmbiguity;
		@Getter
		private final Translation2d[] corners;
		
		private LimelightTarget(int fiducialID, int objectClassID, double detectedObjectConfidence, double yawRads, double pitchRads, double skewRads, Transform3d bestCameraToTag, Transform3d altCameraToTag, double poseAmbiguity, Translation2d[] corners) {
			this.fiducialID = fiducialID;
			this.objectClassID = objectClassID;
			this.detectedObjectConfidence = detectedObjectConfidence;
			this.yawRads = yawRads;
			this.pitchRads = pitchRads;
			this.skewRads = skewRads;
			this.bestCameraToTag = bestCameraToTag;
			this.altCameraToTag = altCameraToTag;
			this.poseAmbiguity = poseAmbiguity;
			this.corners = corners;
		}
	}
}

package frc.robot.subsystems.vision.object;

public class ObjectVision {
	// private final ObjectPipeline[] pipelines;

	// private static final LoggedTunable<Distance> updateDistanceThreshold = LoggedTunable.from("Vision/Object/Updating/Update Distance Threshold", Meters::of, 5);
	// private static final LoggedTunableNumber posUpdatingFilteringFactor =  LoggedTunable.from("Vision/Object/Updating/Pos Updating Filtering Factor", 0.8);
	// private static final LoggedTunableNumber confUpdatingFilteringFactor = LoggedTunable.from("Vision/Object/Confidence/Updating Filtering Factor", 0.5);
	// private static final LoggedTunableNumber confidenceDecayPerSecond =    LoggedTunable.from("Vision/Object/Confidence/Decay Per Second", 3);
	// private static final LoggedTunableNumber priorityPerConfidence =       LoggedTunable.from("Vision/Object/Priority/Priority Per Confidence", 4);
	// private static final LoggedTunableNumber priorityPerDistance =         LoggedTunable.from("Vision/Object/Priority/Priority Per Distance", -2);
	// private static final LoggedTunableNumber acquireConfidenceThreshold =  LoggedTunable.from("Vision/Object/Target Threshold/Acquire", 0.75);
	// private static final LoggedTunableNumber detargetConfidenceThreshold = LoggedTunable.from("Vision/Object/Target Threshold/Detarget", -3);

	//private final ArrayList<TrackedObject> objectMemories = new ArrayList<>(3);

	public ObjectVision(ObjectPipeline... pipelines) {
		System.out.println("[Init ObjectVision] Instantiating ObjectVision");
		// this.pipelines = pipelines;
	}

	public void periodic() {

	}
}

package frc.robot.auto;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;

public class FreeRangeAutoSelectorIOServer implements FreeRangeAutoSelectorIO {
	private static final String toRobotTable = "/FreeRangeSelector/ToRobot/";

	private static final String shouldKeepOutName = "KeepOut/ShouldKeepOut";
	private static final String keepOutTopLeftName = "KeepOut/TopLeft";
	private static final String keepOutBottomRightName = "KeepOut/BottomRight";

	private static final String shouldClimbName = "ShouldClimb";
	private static final String shouldNeutralZoneName = "ShouldNeutralZone";
	private static final String shouldAvoidBumpName = "ShouldAvoidBump";
	private static final String shouldNotAutoIntakeName = "ShouldNotAutoIntake";
	private static final String shouldStopToShootName = "ShouldStopToShoot";

	private final BooleanSubscriber keepOutSubscriber;
	private final DoubleSubscriber topLeftXSubscriber;
	private final DoubleSubscriber topLeftYSubscriber;
	private final DoubleSubscriber bottomRightXSubscriber;
	private final DoubleSubscriber bottomRightYSubscriber;
	private final BooleanSubscriber climbSubscriber;
	private final BooleanSubscriber neutralZoneSubscriber;
	private final BooleanSubscriber avoidBumpSubscriber;
	private final BooleanSubscriber notAutoIntakeSubscriber;
	private final BooleanSubscriber stopToShootSubscriber;

	public FreeRangeAutoSelectorIOServer() {
		System.out.println("[Init] Creating FreeRangeAutoSelectorIOServer");

		WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/free_range_selector");

		var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);

		keepOutSubscriber =
			inputTable
			.getBooleanTopic(shouldKeepOutName)
			.subscribe(true, PubSubOption.keepDuplicates(true));
		topLeftXSubscriber =
			inputTable
			.getDoubleTopic(keepOutTopLeftName + "X")
			.subscribe(0.0, PubSubOption.keepDuplicates(true));
		topLeftYSubscriber =
			inputTable
			.getDoubleTopic(keepOutTopLeftName + "Y")
			.subscribe(0.0, PubSubOption.keepDuplicates(true));
		bottomRightXSubscriber =
			inputTable
			.getDoubleTopic(keepOutBottomRightName + "X")
			.subscribe(0.0, PubSubOption.keepDuplicates(true));
		bottomRightYSubscriber =
			inputTable
			.getDoubleTopic(keepOutBottomRightName + "Y")
			.subscribe(0.0, PubSubOption.keepDuplicates(true));
		neutralZoneSubscriber =
			inputTable
			.getBooleanTopic(shouldNeutralZoneName)
			.subscribe(false, PubSubOption.keepDuplicates(true));
		avoidBumpSubscriber =
			inputTable
			.getBooleanTopic(shouldAvoidBumpName)
			.subscribe(false, PubSubOption.keepDuplicates(true));
		notAutoIntakeSubscriber =
			inputTable
			.getBooleanTopic(shouldNotAutoIntakeName)
			.subscribe(false, PubSubOption.keepDuplicates(true));
		stopToShootSubscriber =
			inputTable
			.getBooleanTopic(shouldStopToShootName)
			.subscribe(false, PubSubOption.keepDuplicates(true));
		climbSubscriber =
			inputTable
			.getBooleanTopic(shouldClimbName)
			.subscribe(true, PubSubOption.keepDuplicates(true));
	}

	@Override
	public void updateInputs(FreeRangeAutoSelectorIOInputs inputs) {
		if (keepOutSubscriber.readQueue().length > 0) {
			inputs.shouldKeepOut = !(boolean) keepOutSubscriber.get();
		}
		if (topLeftXSubscriber.readQueue().length > 0) {
			inputs.keepOutTopLeft[0] = topLeftXSubscriber.get();
		}
		if (topLeftYSubscriber.readQueue().length > 0) {
			inputs.keepOutTopLeft[1] = topLeftYSubscriber.get();
		}
		if (bottomRightXSubscriber.readQueue().length > 0) {
			inputs.keepOutBottomRight[0] = bottomRightXSubscriber.get();
		}
		if (bottomRightYSubscriber.readQueue().length > 0) {
			inputs.keepOutBottomRight[1] = bottomRightYSubscriber.get();
		}
		if (climbSubscriber.readQueue().length > 0) {
			inputs.shouldClimb = (boolean) climbSubscriber.get();
		}
		if (neutralZoneSubscriber.readQueue().length > 0) {
			inputs.shouldNeutralZone = (boolean) neutralZoneSubscriber.get();
		}
		if (avoidBumpSubscriber.readQueue().length > 0) {
			inputs.shouldAvoidBump = (boolean) avoidBumpSubscriber.get();
		}
		if (notAutoIntakeSubscriber.readQueue().length > 0) {
			inputs.shouldNotAutoIntake = (boolean) notAutoIntakeSubscriber.get();
		}
		if (stopToShootSubscriber.readQueue().length > 0) {
			inputs.shouldStopToShoot = (boolean) stopToShootSubscriber.get();
		}
	}
}
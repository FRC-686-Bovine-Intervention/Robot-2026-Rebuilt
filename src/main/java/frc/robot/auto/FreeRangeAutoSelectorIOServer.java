package frc.robot.auto;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;

public class FreeRangeAutoSelectorIOServer implements FreeRangeAutoSelectorIO {
	private static final String toRobotTable = "/FreeRangeSelector/ToRobot/";

	private static final String shouldKeepOutName = "KeepOut/ShouldKeepOut";
	private static final String keepOutTopLeftName = "KeepOut/TopLeft";
	private static final String keepOutBottomRightName = "KeepOut/BottomRight";

	private static final String shouldClimbName = "ShouldClimb";

	private final BooleanSubscriber keepOutSubscriber;
	private final DoubleArraySubscriber topLeftSubscriber;
	private final DoubleArraySubscriber bottomRightSubscriber;

	private final BooleanSubscriber climbSubscriber;

	public FreeRangeAutoSelectorIOServer() {
		System.out.println("[Init] Creating FreeRangeAutoSelectorIOServer");

		WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/free_range_selector");

		var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);

		keepOutSubscriber =
			inputTable
			.getBooleanTopic(shouldKeepOutName)
			.subscribe(true, PubSubOption.keepDuplicates(true));
		topLeftSubscriber =
			inputTable
			.getDoubleArrayTopic(keepOutTopLeftName)
			.subscribe(new double[] {}, PubSubOption.keepDuplicates(true));
		bottomRightSubscriber =
			inputTable
			.getDoubleArrayTopic(keepOutBottomRightName)
			.subscribe(new double[] {}, PubSubOption.keepDuplicates(true));
		climbSubscriber =
			inputTable
			.getBooleanTopic(shouldClimbName)
			.subscribe(true, PubSubOption.keepDuplicates(true));
	}

	@Override
	public void updateInputs(FreeRangeAutoSelectorIOInputs inputs) {
		if (keepOutSubscriber.readQueue().length > 0) {
			inputs.shouldKeepOut = (boolean) keepOutSubscriber.get();
		}
		if (topLeftSubscriber.readQueueValues().length > 0) {
			inputs.keepOutTopLeft = topLeftSubscriber.get();
		}
		if (bottomRightSubscriber.readQueueValues().length > 0) {
			inputs.keepOutTopRight = bottomRightSubscriber.get();
		}
		if (climbSubscriber.readQueue().length > 0) {
			inputs.shouldClimb = (boolean) climbSubscriber.get();
		}
	}
}

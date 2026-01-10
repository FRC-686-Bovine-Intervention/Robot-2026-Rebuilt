package frc.util.faults;

import java.util.Arrays;
import java.util.function.LongConsumer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import frc.util.faults.DeviceFaults.FaultType;

public class DeviceFaultClearer extends LoggedNetworkInput {
	private final StringPublisher namePublisher;
	private final StringPublisher typePublisher;
	private final StringArrayPublisher optionsPublisher;
	private final StringPublisher defaultPublisher;
	private final StringPublisher activePublisher;
	private final StringEntry selectedEntry;
	private final String key;
	private String[] value = new String[0];

	public DeviceFaultClearer(String key) {
		this.key = key;
		var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(this.key);
		this.namePublisher = table.getStringTopic(".name").publish();
		this.typePublisher = table.getStringTopic(".type").publish();
		this.optionsPublisher = table.getStringArrayTopic("options").publish();
		this.defaultPublisher = table.getStringTopic("default").publish();
		this.activePublisher = table.getStringTopic("active").publish();
		this.selectedEntry = table.getStringTopic("selected").getEntry("", PubSubOption.excludeSelf(true));

		Logger.registerDashboardInput(this);

		this.namePublisher.set(this.key);
		this.typePublisher.set("String Chooser");
		this.defaultPublisher.set("");
		this.activePublisher.set("");
		this.selectedEntry.set("");
	}

	public void clear(DeviceFaults faults, LongConsumer clearingFunction, long bitmask) {
		if (faults.isAllOk()) {
			this.optionsPublisher.set(new String[0]);
		} else {
			this.optionsPublisher.set(Arrays
				.stream(faults.getActiveFaults(bitmask))
				.map(FaultType::name)
				.toArray(String[]::new)
			);
		}
		var faultsToClear = DeviceFaults.noneMask;
		for (var string : this.value) {
			var faultType = FaultType.valueOf(string);
			faultsToClear |= faultType.getBitmask();
		}
		if (faultsToClear != DeviceFaults.noneMask) {
			clearingFunction.accept(faultsToClear);
			this.selectedEntry.set("");
		}
	}

	private final LoggableInputs inputs = new LoggableInputs() {
		@Override
		public void toLog(LogTable table) {
			table.put(LoggedNetworkInput.removeSlash(key), value);
		}

		@Override
		public void fromLog(LogTable table) {
			value = table.get(LoggedNetworkInput.removeSlash(key), new String[0]);
		}
	};

	@Override
	public void periodic() {
		if (!Logger.hasReplaySource()) {
			this.value = this.selectedEntry.readQueueValues();
		}
		Logger.processInputs(LoggedNetworkInput.prefix, this.inputs);
	}
}

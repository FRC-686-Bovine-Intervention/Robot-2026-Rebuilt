package frc.util;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class SwitchableChooser extends LoggedNetworkInput {
	private String[] options;
	private String selectedValue;
	private String activeValue;
	private String defaultValue;

	private final StringPublisher namePublisher;
	private final StringPublisher typePublisher;
	private final StringArrayPublisher optionsPublisher;
	private final StringPublisher defaultPublisher;
	private final StringPublisher activePublisher;
	private final LoggedNetworkString selectedInput;
	// private final Map<Integer, String> lastHasChangedValues = new HashMap<>();

	public SwitchableChooser(String name) {
		var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
		namePublisher = table.getStringTopic(".name").publish();
		typePublisher = table.getStringTopic(".type").publish();
		optionsPublisher = table.getStringArrayTopic("options").publish();
		defaultPublisher = table.getStringTopic("default").publish();
		activePublisher = table.getStringTopic("active").publish();
		selectedInput = new LoggedNetworkString("/SmartDashboard/" + name + "/selected");
		Logger.registerDashboardInput(this);

		namePublisher.set(name);
		typePublisher.set("String Chooser");
		setOptions();
	}

	public void periodic() {
		this.selectedValue = getOption(this.selectedInput.get());
	}

	/** Returns the selected option. */
	public String getSelected() {
		return selectedValue;
	}

	public String getActive() {
		return activeValue;
	}

	public String getDefault() {
		return activeValue;
	}

	public String[] getOptions() {
		return options;
	}

	// public boolean hasChanged(int id) {
	//     var currentValue = getSelected();
	//     var lastValue = lastHasChangedValues.get(id);
	//     if (lastValue == null || currentValue != lastValue) {
	//         lastHasChangedValues.put(id, currentValue);
	//         return true;
	//     }

	//     return false;
	// }

	public void setSelected(String selectedOption) {
		if (this.selectedValue == selectedOption) {
			return;
		}
		this.selectedValue = getOption(selectedOption);
		selectedInput.set(this.selectedValue);
	}

	/** Updates the set of available options. */
	public void setOptions(String... options) {
		// if (Arrays.equals(options, this.options)) {
		//     return;
		// }
		this.options = options;
		optionsPublisher.set(this.options);
	}

	public void setActive(String activeOption) {
		this.activeValue = getOption(activeOption);
		activePublisher.set(this.activeValue);
	}

	public void setDefault(String defaultOption) {
		this.defaultValue = defaultOption;
		defaultPublisher.set(this.defaultValue);
	}

	private String getOption(String optiona) {
		for (var innerOption : this.options) {
			if (Objects.equals(optiona, innerOption)) {
				return innerOption;
			}
		}
		return this.defaultValue;
	}
}

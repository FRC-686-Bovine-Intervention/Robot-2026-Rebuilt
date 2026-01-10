package frc.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

public enum Environment {
	Practice,
	Competition,
	Demo,
	;
	private static final String key = "Environment Chooser";
	private static final StringPublisher namePublisher;
	private static final StringPublisher typePublisher;
	private static final StringArrayPublisher optionsPublisher;
	private static final StringPublisher defaultPublisher;
	private static final StringPublisher activePublisher;
	private static final StringEntry selectedEntry;
	private static String[] ntArray;
	private static final LoggableInputs inputs = new LoggableInputs() {
		@Override
		public void toLog(LogTable table) {
			table.put(key, ntArray);
		}

		@Override
		public void fromLog(LogTable table) {
			ntArray = table.get(key, ntArray);
		}
	};

	private static final Map<String, Environment> map;
	private static int selectionPriority;
	private static String selectedName;
	private static Environment selectedValue;

	private static final Alert fms_alert = new Alert("FMS detected, Competition Environment selected", AlertType.kInfo);
	private static final Alert fms_no_comp_alert = new Alert("FMS detected but selected Environment is not Competition", AlertType.kWarning);
	private static final Alert demo_alert = new Alert("Demo Environment selected, Robot functionality restricted", AlertType.kWarning);

	static {
		var practiceName = "Practice";
		var competitionName = "Competition";
		var demoName = "Demo";

		map = new HashMap<>(3);
		map.put(practiceName, Practice);
		map.put(competitionName, Competition);
		map.put(demoName, Demo);

		selectedName = practiceName;

		var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key);
		namePublisher = table.getStringTopic(".name").publish();
		typePublisher = table.getStringTopic(".type").publish();
		optionsPublisher = table.getStringArrayTopic("options").publish();
		defaultPublisher = table.getStringTopic("default").publish();
		activePublisher = table.getStringTopic("active").publish();
		selectedEntry = table.getStringTopic("selected").getEntry(selectedName, PubSubOption.excludeSelf(true));

		namePublisher.set(key);
		typePublisher.set("String Chooser");

		optionsPublisher.set(new String[] {
			practiceName,
			competitionName,
			demoName
		});

		defaultPublisher.set(selectedName);
		activePublisher.set(selectedName);
		selectedEntry.set(selectedName);

		selectionPriority = 0;
	}

	public static void periodic() {
		ntArray = selectedEntry.readQueueValues();
		Logger.processInputs("NetworkInputs", inputs);
		for (var selected : ntArray) {
			selectedName = selected;
			selectionPriority = 2;
		}
		if (selectionPriority <= 1 && DriverStation.isFMSAttached()) {
			selectedName = "Competition";
			selectedEntry.set(selectedName);
			selectionPriority = 1;
		}
		selectedValue = map.get(selectedName);
		activePublisher.set(selectedName);

		fms_alert.set(DriverStation.isFMSAttached() && isCompetition());
		fms_no_comp_alert.set(DriverStation.isFMSAttached() && !isCompetition());
		demo_alert.set(isDemo());
	}

	public static Environment getEnvironment() {
		return selectedValue;
	}
	public static boolean is(Environment is) {
		return is.equals(selectedValue);
	}
	public static boolean isPractice() {
		return is(Practice);
	}
	public static boolean isCompetition() {
		return is(Competition);
	}
	public static boolean isDemo() {
		return is(Demo);
	}

	public static <T> Supplier<T> switchVar(T prac_comp, T demo) {
		return switchVar(prac_comp, prac_comp, demo);
	}
	public static <T> Supplier<T> switchVar(T prac, T comp, T demo) {
		return () -> switch(selectedValue) {
			default -> prac;
			case Competition -> comp;
			case Demo -> demo;
		};
	}
}

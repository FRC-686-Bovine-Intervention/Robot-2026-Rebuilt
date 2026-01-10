package frc.util.loggerUtil.tunables;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.constants.RobotConstants;
/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber implements LoggedTunable<Double> {
	private final double defaultValue;
	private final LoggedNetworkNumber dashboardNumber;
	private final Map<Integer, Double> lastHasChangedValues;

	/**
	 * Create a new LoggedTunableNumber with the default value
	 *
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 */
	public LoggedTunableNumber(String key, double defaultValue) {
		this.defaultValue = defaultValue;
		if (RobotConstants.tuningMode) {
			this.dashboardNumber = new LoggedNetworkNumber(LoggedTunable.TABLE_KEY + "/" + key, this.defaultValue);
			this.lastHasChangedValues = new HashMap<>();
		} else {
			this.dashboardNumber = null;
			this.lastHasChangedValues = null;
		}
	}

	/**
	 * Get the current value, from dashboard if available and in tuning mode.
	 *
	 * @return The current value
	 */
	public double getAsDouble() {
		return RobotConstants.tuningMode ? this.dashboardNumber.get() : this.defaultValue;
	}

	@Override
	public Double get() {
		return this.getAsDouble();
	}

	@Override
	public boolean hasChanged(int id) {
		if (!RobotConstants.tuningMode) {
			return false;
		}
		double currentValue = this.getAsDouble();
		Double lastValue = this.lastHasChangedValues.get(id);
		if (lastValue == null || currentValue != lastValue) {
			this.lastHasChangedValues.put(id, currentValue);
			return true;
		}
		return false;
	}
}

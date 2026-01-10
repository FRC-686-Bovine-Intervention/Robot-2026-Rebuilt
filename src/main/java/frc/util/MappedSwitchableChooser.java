package frc.util;

import java.util.Map;
import java.util.Map.Entry;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class MappedSwitchableChooser<T> {
	private Map<String, T> options;

	private final SwitchableChooser chooser;

	public MappedSwitchableChooser(String name, Map<String, T> options, T defaultOption) {
		this.chooser = new SwitchableChooser(name);
		setOptions(options);
		setDefault(defaultOption);
	}

	/** Updates the set of available options. */
	public void setOptions(Map<String, T> options) {
		this.options = options;
		chooser.setOptions(this.options.keySet().toArray(String[]::new));
	}

	public void setSelected(T selectedValue) {
		this.chooser.setSelected(getKey(selectedValue));
	}

	public void setActive(T activeValue) {
		this.chooser.setActive(getKey(activeValue));
	}

	public void setDefault(T defaultValue) {
		this.chooser.setDefault(getKey(defaultValue));
	}

	public Map<String, T> getOptions() {
		return options;
	}

	/** Returns the selected option. */
	public T getSelected() {
		return this.options.get(this.chooser.getSelected());
	}

	public T getActive() {
		return this.options.get(this.chooser.getActive());
	}

	public T getDefault() {
		return this.options.get(this.chooser.getDefault());
	}

	private String getKey(T value) {
		return this.options.entrySet().stream().filter((e) -> e.getValue().equals(value)).map(Entry::getKey).findAny().orElse(null);
	}
}

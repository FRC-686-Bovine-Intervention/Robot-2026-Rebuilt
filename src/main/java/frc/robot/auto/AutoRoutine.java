package frc.robot.auto;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class AutoRoutine {
	public final String name;
	public final List<AutoQuestion<?>> questions;

	public AutoRoutine(String name, List<AutoQuestion<?>> questions) {
		this.name = name;
		this.questions = questions;
	}

	public abstract Command generateCommand();

	public static abstract class AutoQuestion<T> {
		public final String name;
		private Settings<T> settings;
		private String response;

		public AutoQuestion(String name) {
			this.name = name;
		}

		protected abstract Settings<T> generateSettings();

		public Settings<T> updateSettings() {
			this.settings = this.generateSettings();
			return this.settings;
		}
		public Settings<T> getSettings() {
			return settings;
		}

		public T getResponse() {
			return this.getSettings().options.get(this.response);
		}

		public String setResponse(String newResponse) {
			if (!this.settings.options().containsKey(newResponse)) {
				newResponse = this.settings.defaultOption().getKey();
			}
			this.response = newResponse;
			return newResponse;
		}

		public String[] getOptionNames() {
			return settings.options().keySet().stream().toArray(String[]::new);
		}

		public boolean equals(Object obj) {
			if (obj == null || !(obj instanceof AutoQuestion)) {
				return false;
			}

			AutoQuestion<?> other = (AutoQuestion<?>) obj;
			return this.name.equals(other.name) && Arrays.equals(this.getOptionNames(), other.getOptionNames());
		}

		public static record Settings<T>(Map<String, T> options, Map.Entry<String, T> defaultOption) {
			public static <T> Map.Entry<String, T> option(String name, T value) {
				return Map.entry(new String(name), value);
			}

			@SafeVarargs
			public static <T> Settings<T> from(Map.Entry<String, T> defaultOption, Map.Entry<String, T>... options) {
				var map = new LinkedHashMap<String, T>();
				for (var option : options) {
					map.put(option.getKey(), option.getValue());
				}
				return new Settings<T>(map, defaultOption);
			}

			public static <T> Settings<T> from(Map.Entry<String, T> defaultOption, Map<String, T> options) {
				return new Settings<T>(options, defaultOption);
			}

			public static <T> Settings<T> empty() {
				return new Settings<T>(Map.of(), null);
			}

			public String[] getOptionNames() {
				return options.keySet().stream().toArray(String[]::new);
			}
		}

		/**
		 * Not actually deprecated, just used to mark testing code that should be replaced later with concrete code
		 */
		@Deprecated(forRemoval = false)
		public static AutoQuestion<Boolean> testBoolean(String name, boolean defaultOption) {
			return new AutoQuestion<Boolean>(name) {
				private final Map.Entry<String, Boolean> falseOption = Settings.option("False", false);
				private final Map.Entry<String, Boolean> trueOption = Settings.option("True", true);
				@Override
				protected Settings<Boolean> generateSettings() {
					return Settings.from(
						(defaultOption) ? (
							trueOption
						) : (
							falseOption
						),
						falseOption, trueOption
					);
				}
			};
		}
	}
}

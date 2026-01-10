package frc.util.loggerUtil;

import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggerUtil {

	public static <T> T[] toArray(Optional<T> optional, IntFunction<T[]> generator) {
		T[] array;
		if (optional.isPresent()) {
			array = generator.apply(1);
			array[0] = optional.get();
		} else {
			array = generator.apply(0);
		}
		return array;
	}

	public static void toLogArray(LogTable table, LoggableInputs... array) {
		table.put("length", array.length);
		for (int i = 0; i < array.length; i++) {
			array[i].toLog(table.getSubtable(Integer.toString(i)));
		}
	}
	public static <T extends LoggableInputs> T[] fromLogArray(LogTable table, Supplier<T> constructor, IntFunction<T[]> arrayGenerator) {
		var array = arrayGenerator.apply(table.get("length", 0));
		for (int i = 0; i < array.length; i++) {
			array[i] = constructor.get();
			array[i].fromLog(table.getSubtable(Integer.toString(i)));
		}
		return array;
	}

	public static <T extends LoggableInputs> void toLogOptional(LogTable table, Optional<T> optional) {
		if (optional.isEmpty()) {
			table.put("length", 0);
		} else {
			table.put("length", 1);
			optional.get().toLog(table.getSubtable("0"));
		}
	}
	public static <T extends LoggableInputs> Optional<T> fromLogOptional(LogTable table, Supplier<T> constructor) {
		if (table.get("length", 0) <= 0) {
			return Optional.empty();
		} else {
			var ret = constructor.get();
			ret.fromLog(table.getSubtable("0"));
			return Optional.of(ret);
		}
	}
}

package frc.util.misc;

import java.util.function.Supplier;

public class FunctionalUtil {
	public static <T> Supplier<T> evalNow(T val) {
		return () -> val;
	}
}

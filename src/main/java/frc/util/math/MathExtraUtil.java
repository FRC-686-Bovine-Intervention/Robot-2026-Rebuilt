package frc.util.math;

import java.util.Arrays;

public class MathExtraUtil {
	public static double average(double a, double b) {
		return (a + b) / 2.0;
	}
	public static double average(double... a) {
		return Arrays.stream(a).average().orElse(0);
	}

	public static boolean isWithin(double value, double min, double max) {
		return value >= min && value <= max;
	}

	public static double dotProduct(double[] a, double[] b) {
		double output = 0;
		for (int i = 0; i < a.length; i++) {
			output += a[i]*b[i];
		}
		return output;
	}
}

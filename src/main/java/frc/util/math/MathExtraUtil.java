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

	public static double[] addVectors(double[] a, double[] b) {
		double[] longestVec;
		double[] shortestVec;
		if (a.length < b.length) {
			longestVec = b;
			shortestVec = a;
		} else {
			longestVec = a;
			shortestVec = b;
		}
		double[] output = longestVec.clone();
		for (int i = 0; i < shortestVec.length; i++) {
			output[i] += shortestVec[i];
		}
		return output;
	}

	public static double[] scalarMultiply(double[] a, double scalar) {
		double[] output = new double[a.length];
		for (int i = 0; i < a.length; i++) {
			output[i] = a[i] * scalar;
		}
		return output;
	}

	public static double[] matchVectorLength2d(double[] x, double[] reference) {
		double referenceLength = Math.sqrt(Math.pow(reference[0], 2) + Math.pow(reference[1], 2));
		double xLength = Math.sqrt(Math.pow(x[0], 2) + Math.pow(x[1], 2));

		return scalarMultiply(x, referenceLength/xLength);
	}
}

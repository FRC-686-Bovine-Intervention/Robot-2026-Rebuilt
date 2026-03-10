package frc.util.math;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

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

	public static void addVectors(double[] out, double[] a, double[] b) {
		double[] longestVec;
		double[] shortestVec;
		if (a.length < b.length) {
			longestVec = b;
			shortestVec = a;
		} else {
			longestVec = a;
			shortestVec = b;
		}
		for (int i = 0; i < shortestVec.length; i++) {
			out[i] = longestVec[i] + shortestVec[i];
		}
	}

	public static void scalarMultiply(double[] out, double[] a, double scalar) {
		for (int i = 0; i < out.length; i++) {
			out[i] = a[i] * scalar;
		}
	}

	public static void putVector(double[] x, double... values) {
		int lowestLength = x.length < values.length ? x.length : values.length;
		for (int i = 0; i < lowestLength; i++) {
			x[i] = values[i];
		}
	}

	public static double norm(double[] v) {
		double total = 0;
		for (int i = 0; i < v.length; i++) {
			total += v[i] * v[i];
		}

		return Math.sqrt(total);
	}

	public static void transformBy(double[] out, Pose2d pose, Transform3d transform) {
		double theta = pose.getRotation().getRadians();
		out[0] = pose.getX() + transform.getX() * Math.cos(theta) - transform.getY() * Math.sin(theta);
		out[1] = pose.getY() + transform.getX() * Math.sin(theta) + transform.getY() * Math.cos(theta);
		out[2] = transform.getZ();
	}
}

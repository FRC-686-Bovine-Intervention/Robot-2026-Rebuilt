package frc.util.math.polynomial;

public class Vector2Polynomial3rdDegree {
	private final double[] b;
	private final double[][] A1;
	private final double[][][] A2;
	private final double[][][][] A3;

	public Vector2Polynomial3rdDegree(double[] b, double[][] A1, double[][][] A2, double[][][][] A3) {
		this.b = b;
		this.A1 = A1;
		this.A2 = A2;
		this.A3 = A3;
	}

	public double[] evaluate(double[] x) {
		double[] result = b;
		result = addDoubleArrays(matrix2x2TimesVec2(x, A1), result);
		result = addDoubleArrays(tensor2x2x2TimesVec2(x, A2), result);
		result = addDoubleArrays(tensor2x2x2x2TimesVec2(x, A3), result);
		return result;
	}

	private double[] matrix2x2TimesVec2(double[] x, double[][] matrix) {
		return new double[] {
			matrix[0][0]*x[0] + matrix[0][1]*x[1],
			matrix[1][0]*x[0] + matrix[1][1]*x[1]
		};
	}

	private double[] tensor2x2x2TimesVec2(double[] x, double[][][] tensor) {
		return new double[] {
			tensor[0][0][0]*x[0]*x[0] + tensor[0][0][1]*x[0]*x[1] + tensor[0][1][0]*x[0]*x[1] + tensor[0][1][1]*x[1]*x[1],
			tensor[1][0][0]*x[0]*x[0] + tensor[1][0][1]*x[0]*x[1] + tensor[1][1][0]*x[0]*x[1] + tensor[1][1][1]*x[1]*x[1]
		};
	}

	private double[] tensor2x2x2x2TimesVec2(double[]x, double[][][][] tensor) {
		return new double[] {
			tensor[0][0][0][0]*x[0]*x[0]*x[0] + tensor[0][0][0][1]*x[0]*x[0]*x[1] + tensor[0][0][1][0]*x[0]*x[0]*x[1] + tensor[0][0][1][1]*x[0]*x[1]*x[1] + tensor[0][1][0][0]*x[0]*x[0]*x[1] + tensor[0][1][0][1]*x[0]*x[1]*x[1] + tensor[0][1][1][0]*x[0]*x[1]*x[1] + tensor[0][1][1][1]*x[1]*x[1]*x[1],
			tensor[1][0][0][0]*x[0]*x[0]*x[0] + tensor[1][0][0][1]*x[0]*x[0]*x[1] + tensor[1][0][1][0]*x[0]*x[0]*x[1] + tensor[1][0][1][1]*x[0]*x[1]*x[1] + tensor[1][1][0][0]*x[0]*x[0]*x[1] + tensor[1][1][0][1]*x[0]*x[1]*x[1] + tensor[1][1][1][0]*x[0]*x[1]*x[1] + tensor[1][1][1][1]*x[1]*x[1]*x[1]
		};
	}

	private double[] addDoubleArrays(double[] x, double[] y) {
		double[] result = new double[x.length];
		for (int i = 0; i < x.length; i++) {
			result[0] = x[0] + y[0];
			result[1] = x[1] + y[1];
		}
		return result;
	}
}

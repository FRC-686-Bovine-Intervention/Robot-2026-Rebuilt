package frc.util.math.polynomial;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class PolynomialRegressionBuilder {
	private List<Translation2d> points;
	private int degree = 0;

	public PolynomialRegressionBuilder(int degree) {
		this.degree = degree;
	}

	public PolynomialRegressionBuilder() {}

	public PolynomialRegressionBuilder addPoint(double x, double y) {
		points.add(new Translation2d(x, y));
		return this;
	}

	public PolynomialRegression build() {
		if (degree == 0) {
			degree = points.size() - 1;
		}
		double[] x = new double[points.size()];
		double[] y = new double[points.size()];
		for (int i = 0; i < points.size(); i++) {
			x[i] = points.get(i).getX();
			y[i] = points.get(i).getY();
		}
		return new PolynomialRegression(x, y, degree);
	}
}

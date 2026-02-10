package frc.util.math.polynomial;

import java.io.FileNotFoundException;
import java.io.FileReader;

import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

public class TwoVariablePolynomial3rdDegree {
	private final double[] coefficients;

	public TwoVariablePolynomial3rdDegree(double[] coefficients) {
		this.coefficients = coefficients;
	}

	public double evaluate(double x, double y) {
		return coefficients[0] + coefficients[1]*x + coefficients[2]*y + coefficients[3]*x*x + coefficients[4]*x*y + coefficients[5]*y*y + coefficients[6]*x*x*x + coefficients[7]*x*x*y + coefficients[8]*x*y*y + coefficients[9]*y*y*y;
	}

	public static TwoVariablePolynomial3rdDegree from(String jsonPath) {
		Gson parser = new Gson();

		try {
			System.out.println("Loading polynomial from " + jsonPath);
			return parser.fromJson(new FileReader(jsonPath), TwoVariablePolynomial3rdDegree.class);
		} catch (JsonSyntaxException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		} catch (JsonIOException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		}
		return null;
	}
}

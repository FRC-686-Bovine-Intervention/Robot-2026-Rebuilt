package frc.util.math.polynomial;

import java.util.ArrayList;
import java.util.List;

import frc.util.math.tensor.Tensor;

public class VectorPolynomial {
	private final int dim;
	private final List<Tensor> coeffs;

	public VectorPolynomial(int dim) {
		this.dim = dim;
		this.coeffs = new ArrayList<>();
	}

	public void addCoefficientTensor(Tensor tensor) {
		if (tensor.shape()[0] != dim) {
			throw new IllegalArgumentException("First dimension of tensor must match dim");
		}
		int rank = tensor.rank();
		for (int i = 1; i < rank; i++) {
			if (tensor.shape()[i] != dim) {
				throw new IllegalArgumentException("Coefficient tensor input axes must match dim");
			}
		}
		coeffs.add(tensor);
	}

	public double[] evaluate(double[] x) {
		if (x.length != dim) {
			throw new IllegalArgumentException(
				"Input dimension " + x.length + " does not match output dimension " + dim
			);
		}

		double[] total = new double[dim];

		for (Tensor tensor : coeffs) {
			Tensor tmp = tensor;

			for (int i = 0; i < tensor.rank() - 1; i++) {
				tmp = tmp.contractLastWith(x);
			}

			double[] term = tmp.toVector();

			for (int i = 0; i < dim; i++) {
				total[i] += term[i];
			}
		}

		return total;
	}
}

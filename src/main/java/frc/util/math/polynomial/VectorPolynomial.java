package frc.util.math.polynomial;

import java.util.ArrayList;
import java.util.List;

import frc.util.math.tensor.Tensor;
import frc.util.math.tensor.TensorUtils;

public class VectorPolynomial {
    private final int inputDim;
    private final int outputDim;
    private final List<Tensor> coeffs;

    public VectorPolynomial(int inputDim, int outputDim) {
        this.inputDim = inputDim;
        this.outputDim = outputDim;
        this.coeffs = new ArrayList<>();
    }

    public void addCoefficientTensor(Tensor tensor) {
        if (tensor.shape()[0] != outputDim) {
            throw new IllegalArgumentException("First dimension of tensor must match outputDim");
        }
        int rank = tensor.rank();
        for (int i = 1; i < rank; i++) {
            if (tensor.shape()[i] != inputDim) {
                throw new IllegalArgumentException("Coefficient tensor input axes must match inputDim");
            }
        }
        coeffs.add(tensor);
    }

    private Tensor evaluateTensor(double[] x) {
        if (x.length != inputDim) throw new IllegalArgumentException("Input vector length mismatch");

        Tensor result = new Tensor(outputDim);

        Tensor Ak = coeffs.get(0);
        for (int i = 0; i < outputDim; i++) {
            result.set(result.get(i) + Ak.get(i), i);
        }

        for (int deg = 1; deg < coeffs.size(); deg++) {
            Ak = coeffs.get(deg);
            Tensor Xk = TensorUtils.tensorPower(x, deg + 1);
            Tensor term = Ak.times(Xk);

            for (int i = 0; i < outputDim; i++) {
                result.set(result.get(i) + term.get(i), i);
            }
        }

        return result;
    }

    public double[] evaluate(double[] x) {
        var evaluation = evaluateTensor(x);
        var val = new double[outputDim];
        for (int i = 0; i < evaluation.shape()[0]; i++) {
            val[i] = evaluation.get(i);
        }
        return val;
    }
}

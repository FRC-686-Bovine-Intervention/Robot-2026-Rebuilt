package frc.util.math.tensor;

public class TensorUtils {
    
    public static Tensor tensorPower(double[] x, int n) {
        if (n < 1) throw new IllegalArgumentException("n must be >= 1");

        int dim = x.length;
        int[] shape = new int[n];
        for (int i = 0; i < n; i++) shape[i] = dim;

        Tensor t = new Tensor(shape);

        fillTensorPower(t, x, n, 0, 1, new int[n]);

        return t;
    }

    private static void fillTensorPower(Tensor t, double[] x, int n, int index, double accum, int[] indices) {
        if (index == n) {
            t.set(accum, indices);
        }

        for (int i = 0; i < x.length; i++) {
            indices[index] = i;
            fillTensorPower(t, x, n, index + 1, accum * x[i], indices);
        }
    }
}
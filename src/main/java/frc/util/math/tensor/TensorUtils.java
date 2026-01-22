package frc.util.math.tensor;

public class TensorUtils {
    public static void contractRecursive(
        Tensor T,
        double[] x,
        Tensor result,
        int[] idx,
        int dim
    ) {
        if (dim == idx.length) {
            double sum = 0.0;

            int[] tIdx = new int[T.rank()];
            System.arraycopy(idx, 0, tIdx, 0, idx.length);

            for (int p = 0; p < x.length; p++) {
                tIdx[tIdx.length - 1] = p;
                sum += T.get(tIdx) * x[p];
            }

            result.set(sum, idx);
            return;
        }

        for (int i = 0; i < result.shape()[dim]; i++) {
            idx[dim] = i;
            contractRecursive(T, x, result, idx, dim + 1);
        }
    }
}
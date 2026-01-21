package frc.util.math.tensor;

public class Tensor {
    private final int[] dims;
    private final int[] strides;
    private final double[] data;

    public Tensor(int... dims) {
        this.dims = dims;
        this.strides = new int[dims.length];
        int totalSize = 1;
        for (int i = dims.length -1; i >= 0; i--) {
            strides[i] = totalSize;
            totalSize *= dims[i];
        }
        data = new double[totalSize];
    }

    private int getFlatIndex(int... indices) {
        if (indices.length != dims.length) { throw new IllegalArgumentException("Wrong number of indices"); }
        int idx = 0;
        for (int i = 0; i < dims.length; i++) {
            if (indices[i] < 0 || indices[i] >= dims[i]) { throw new IllegalArgumentException("Index out of bounds"); }
            idx += indices[i] * strides[i];
        }
        return idx;
    }

    public double get(int... indices) {
        return data[getFlatIndex(indices)];
    }

    public void set(double value, int... indices) {
        data[getFlatIndex(indices)] = value;
    }

    public int[] shape() {
        return dims;
    }

    public int rank() {
        return dims.length;
    }

    public int size() {
        return data.length;
    }

    public Tensor times(Tensor other) {
        int kA = this.dims[this.dims.length - 1];
        int kB = other.dims[0];
        if (kA != kB) throw new IllegalArgumentException("Inner dimensions must match: " + kA + " != " + kB);

        int newRank = this.rank() + other.rank() - 2;
        int[] newShape = new int[newRank];
        System.arraycopy(this.dims, 0, newShape, 0, this.dims.length - 1);
        System.arraycopy(other.dims, 1, newShape, this.dims.length - 1, other.dims.length - 1);

        Tensor result = new Tensor(newShape);

        multiplyRecursive(this, other, result, new int[newShape.length], 0, kA);

        return result;
    }

    private static void multiplyRecursive(Tensor A, Tensor B, Tensor result, int[] idxResult, int dim, int k) {
        if (dim == idxResult.length) {

            double sum = 0;
            for (int p = 0; p < k; p++) {
                int[] idxA = new int[A.rank()];
                System.arraycopy(idxResult, 0, idxA, 0, A.rank() - 1);
                idxA[idxA.length - 1] = p;

                int[] idxB = new int[B.rank()];
                idxB[0] = p;
                System.arraycopy(idxResult, A.rank() - 1, idxB, 1, B.rank() - 1);

                sum += A.get(idxA) * B.get(idxB);
            }
            result.set(sum, idxResult);
            return;
        }

        for (int i = 0; i < result.shape()[dim]; i++) {
            idxResult[dim] = i;
            multiplyRecursive(A, B, result, idxResult, dim + 1, k);
        }
    }
}

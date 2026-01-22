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
    
    public double[] toVector() {
        if (this.dims.length > 1) {
            throw new IllegalArgumentException("Must be a vector-shaped tensor");
        }
        double[] result = new double[this.dims[0]];
        for (int i = 0; i < this.dims[0]; i++) {
            result[i] = this.get(i);
        }
        return result;
    }

    public Tensor contractLastWith(double[] x) {
        int n = dims[dims.length - 1];
        if (x.length != n) {
            throw new IllegalArgumentException(
                "Vector length " + x.length + " does not match last tensor dimension " + n
            );
        }

        int[] newShape = new int[dims.length - 1];
        System.arraycopy(dims, 0, newShape, 0, newShape.length);

        Tensor result = new Tensor(newShape);

        TensorUtils.contractRecursive(this, x, result, new int[newShape.length], 0);

        return result;
    }
}

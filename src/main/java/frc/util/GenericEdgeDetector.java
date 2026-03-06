package frc.util;

public class GenericEdgeDetector<T> {
	private T prevVal;
	private T risingEdge;
	private T fallingEdge;

	public GenericEdgeDetector() {
		this(null);
	}
	public GenericEdgeDetector(T startingValue) {
		this.prevVal = startingValue;
	}

	public void update(T value) {
		if (value != this.prevVal) {
			this.risingEdge = value;
			this.fallingEdge = this.prevVal;
		} else {
			this.risingEdge = null;
			this.fallingEdge = null;
		}
		this.prevVal = value;
	}

	public T getValue() {
		return this.prevVal;
	}
	public T risingEdge() {
		return this.risingEdge;
	}
	public T fallingEdge() {
		return this.fallingEdge;
	}
	public boolean changed() {
		return this.risingEdge != null || this.fallingEdge != null;
	}
}

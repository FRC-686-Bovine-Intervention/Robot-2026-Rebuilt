package frc.util;

public class EdgeDetector {
	private boolean prevVal;
	private boolean risingEdge;
	private boolean fallingEdge;

	public EdgeDetector() {
		this(false);
	}
	public EdgeDetector(boolean startingValue) {
		prevVal = startingValue;
	}

	public void update(boolean value) {
		risingEdge = false;
		fallingEdge = false;
		if(value && !prevVal) {
			risingEdge = true;
		}
		if(!value && prevVal) {
			fallingEdge = true;
		}
		prevVal = value;
	}

	public boolean getValue() {
		return prevVal;
	}
	public boolean risingEdge() {
		return risingEdge;
	}
	public boolean fallingEdge() {
		return fallingEdge;
	}
	public boolean changed() {
		return risingEdge || fallingEdge;
	}
}

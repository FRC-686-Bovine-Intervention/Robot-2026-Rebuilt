package frc.util.led.animation;

import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class FlashingAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final DoubleFunction<Color> gradient;
	private final DoubleUnaryOperator tilingFunction;

	public FlashingAnimation(LEDStrip strip, DoubleUnaryOperator tilingFunction, DoubleFunction<Color> gradient) {
		this.strip = strip;
		this.gradient = gradient;
		this.tilingFunction = tilingFunction;
	}

	@Override
	public void apply() {
		this.strip.apply(this.gradient.apply(this.tilingFunction.applyAsDouble(Timer.getTimestamp())));
	}
}

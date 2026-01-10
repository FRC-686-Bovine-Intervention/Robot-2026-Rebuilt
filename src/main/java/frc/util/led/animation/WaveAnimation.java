package frc.util.led.animation;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class WaveAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final DoubleFunction<Color> gradient;
	private final DoubleBinaryOperator tilingFunction;

	public WaveAnimation(LEDStrip strip, DoubleBinaryOperator tilingFunction, DoubleFunction<Color> gradient) {
		this.strip = strip;
		this.gradient = gradient;
		this.tilingFunction = tilingFunction;
	}

	@Override
	public void apply() {
		strip.apply((pos) -> gradient.apply(tilingFunction.applyAsDouble(Timer.getTimestamp(), pos)));
	}
}

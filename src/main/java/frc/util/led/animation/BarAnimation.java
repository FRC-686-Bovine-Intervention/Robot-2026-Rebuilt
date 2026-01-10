package frc.util.led.animation;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class BarAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final DoubleFunction<Color> gradient;
	private double barPos;

	public void setPos(double barPos) {
		this.barPos = barPos;
	}

	public BarAnimation(LEDStrip strip, DoubleFunction<Color> gradient) {
		this.strip = strip;
		this.gradient = gradient;
	}

	@Override
	public void apply() {
		this.strip.foreach((i) -> {
			var pixelPos = (double) i / this.strip.getLength();
			if (pixelPos <= barPos) {
				this.strip.setLED(i, this.gradient.apply(pixelPos));
			}
		});
	}
}

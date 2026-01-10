package frc.util.led.animation;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class FillAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final Color color;

	public FillAnimation(LEDStrip strip, Color color) {
		this.strip = strip;
		this.color = color;
	}

	public void apply() {
		strip.apply(color);
	}
}

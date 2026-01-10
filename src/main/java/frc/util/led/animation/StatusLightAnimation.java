package frc.util.led.animation;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class StatusLightAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final Color falseColor;
	private final Color trueColor;
	private boolean status;

	public StatusLightAnimation(LEDStrip strip, Color falseColor, Color trueColor) {
		this.strip = strip;
		this.falseColor = falseColor;
		this.trueColor = trueColor;
	}

	public void setStatus(boolean status) {
		this.status = status;
	}

	public void apply() {
		strip.apply(status ? trueColor : falseColor);
	}
}

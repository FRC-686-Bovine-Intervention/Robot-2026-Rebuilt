package frc.util.led.strips.adapters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class SubStrip implements LEDStrip {
	private final LEDStrip strip;
	private final int startIndex;
	private final int endIndex;

	public SubStrip(int startIndex, LEDStrip strip) {
		this(startIndex, strip.getLength(), strip);
	}
	public SubStrip(int startIndex, int endIndex, LEDStrip strip) {
		this.startIndex = Math.max(startIndex, 0);
		this.endIndex = Math.min(endIndex, strip.getLength());
		this.strip = strip;
	}

	@Override
	public int getLength() {
		return this.endIndex - this.startIndex;
	}

	@Override
	public void setLED(int ledIndex, Color color) {
		this.strip.setLED(MathUtil.clamp(ledIndex, 0, getLength() - 1) + this.startIndex, color);
	}
}

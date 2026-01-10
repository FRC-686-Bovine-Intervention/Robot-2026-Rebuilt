package frc.util.led.strips.adapters;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class ReversedStrip implements LEDStrip {
	private final LEDStrip strip;

	public ReversedStrip(LEDStrip strip) {
		this.strip = strip;
	}

	@Override
	public LEDStrip reverse() {
		return this.strip;
	}

	@Override
	public int getLength() {
		return this.strip.getLength();
	}

	@Override
	public void setLED(int ledIndex, Color color) {
		this.strip.setLED(getLength() - ledIndex - 1, color);
	}
}

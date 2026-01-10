package frc.util.led.strips.adapters;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class ConcatenatedStrip implements LEDStrip {
	private final LEDStrip[] strips;
	private final int length;

	public ConcatenatedStrip(LEDStrip... strips) {
		this.strips = strips;
		this.length = Arrays.stream(this.strips).mapToInt(LEDStrip::getLength).sum();
	}

	@Override
	public LEDStrip concat(LEDStrip... strips) {
		LEDStrip[] newStrips = new LEDStrip[strips.length + this.strips.length];
		IntStream.range(0, this.strips.length).forEach((i) -> newStrips[i] = this.strips[i]);
		IntStream.range(0, strips.length).forEach((i) -> newStrips[i + this.strips.length] = strips[i]);
		return new ConcatenatedStrip(newStrips);
	}

	@Override
	public int getLength() {
		return this.length;
	}

	@Override
	public void setLED(int ledIndex, Color color) {
		int accumLength = 0;
		for (LEDStrip strip : this.strips) {
			int stripLength = strip.getLength();
			accumLength += stripLength;
			if (accumLength > ledIndex) {
				accumLength -= stripLength;
				ledIndex -= accumLength;
				strip.setLED(ledIndex, color);
				break;
			}
		}
	}
}

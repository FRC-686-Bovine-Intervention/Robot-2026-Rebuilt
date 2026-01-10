package frc.util.led.strips.adapters;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class ParallelStrip implements LEDStrip {
	private final LEDStrip[] strips;
	private final int length;

	public ParallelStrip(LEDStrip... strips) {
		this.strips = strips;
		this.length = Arrays.stream(this.strips).mapToInt(LEDStrip::getLength).max().orElse(0);
	}

	@Override
	public LEDStrip parallel(LEDStrip... strips) {
		LEDStrip[] newStrips = new LEDStrip[strips.length + this.strips.length];
		IntStream.range(0, this.strips.length).forEach((i) -> newStrips[i] = this.strips[i]);
		IntStream.range(0, strips.length).forEach((i) -> newStrips[i + this.strips.length] = strips[i]);
		return new ParallelStrip(newStrips);
	}

	@Override
	public int getLength() {
		return this.length;
	}

	@Override
	public void setLED(int ledIndex, Color color) {
		for (LEDStrip strip : this.strips) {
			strip.setLED(ledIndex, color);
		}
	}
}

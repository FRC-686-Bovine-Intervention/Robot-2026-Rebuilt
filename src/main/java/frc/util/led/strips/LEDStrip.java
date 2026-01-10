package frc.util.led.strips;

import java.util.function.DoubleFunction;
import java.util.function.IntConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.adapters.ConcatenatedStrip;
import frc.util.led.strips.adapters.ParallelStrip;
import frc.util.led.strips.adapters.ReversedStrip;
import frc.util.led.strips.adapters.SubStrip;

public interface LEDStrip {
	public int getLength();
	public static int getLength(LEDStrip[] strips) {
		var acc = 0;
		for (var strip : strips) {
			acc += strip.getLength();
		}
		return acc;
	}

	public void setLED(int ledIndex, Color color);

	public default void foreach(IntConsumer function) {
		for (int i = 0; i < this.getLength(); i++) {
			function.accept(i);
		}
	}
	public default void clear() {
		this.foreach((int i) -> this.setLED(i, Color.kBlack));
	}
	public default void apply(DoubleFunction<Color> gradient) {
		this.foreach((i) -> this.setLED(i, gradient.apply((double) i / this.getLength())));
	}
	public default void apply(Supplier<Color> fill) {
		this.foreach((i) -> this.setLED(i, fill.get()));
	}
	public default void apply(Color fill) {
		this.foreach((i) -> this.setLED(i, fill));
	}

	public default LEDStrip concat(LEDStrip... strips) {
		return new ConcatenatedStrip(this).concat(strips);
	}
	public default LEDStrip substrip(int startIndex) {
		return new SubStrip(startIndex, this);
	}
	public default LEDStrip substrip(int startIndex, int endIndex) {
		return new SubStrip(startIndex, endIndex, this);
	}
	public default LEDStrip reverse() {
		return new ReversedStrip(this);
	}
	public default LEDStrip parallel(LEDStrip... strips) {
		return new ParallelStrip(this).parallel(strips);
	}
}

package frc.util.led.functions;

import java.util.function.DoubleUnaryOperator;

@FunctionalInterface
public interface WaveFunction extends DoubleUnaryOperator {
	public static final WaveFunction Modulo = (x) -> (x % 1 + 1) % 1;
	public static final WaveFunction Sawtooth = (x) -> 1 - Math.abs(((x % 2 + 2) % 2) - 1);
	public static final WaveFunction Sinusoidal = (x) -> 0.5 * (Math.sin(Math.PI * (x - 0.5)) + 1);

	public default WaveFunction frequency(double frequency) {
		return (x) -> this.applyAsDouble(x * frequency);
	}
}

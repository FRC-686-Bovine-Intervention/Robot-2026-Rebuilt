package frc.util.led.functions;

import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.IntStream;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.util.Color;

public class Gradient implements DoubleFunction<Color> {
	public static final DoubleFunction<Color> blackToWhite = (double x) -> new Color(x,x,x);
	public static final DoubleFunction<Color> rainbow = (double x) -> {
		final DoubleUnaryOperator sinusoid = (double X) -> Math.max(Math.cos(2 * Math.PI * X) * (2.0 / 3) + (1.0 / 3), 0);
		return new Color(
			sinusoid.applyAsDouble(x - 0.0/3),
			sinusoid.applyAsDouble(x - 1.0/3),
			sinusoid.applyAsDouble(x - 2.0/3)
		);
	};
	public static final DoubleFunction<Color> hueShift = (double x) -> Color.fromHSV((int)(x * 180),255,255);

	private final InterpolatingTreeMap<Double, Color> interpolator;

	public Gradient(Color color) {
		this(new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), InterpolationFunction.linear));
		this.interpolator.put(0.0, color);
	}
	public Gradient(Interpolator<Color> interpolator, Color... colors) {
		this(new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), interpolator));
		IntStream.range(0, colors.length).forEachOrdered((i) -> this.interpolator.put((double) i / colors.length, colors[i]));
	}
	public Gradient(InterpolatingTreeMap<Double, Color> map) {
		this.interpolator = map;
	}

	@Override
	public Color apply(double x) {
		return interpolator.get(x);
	}
}

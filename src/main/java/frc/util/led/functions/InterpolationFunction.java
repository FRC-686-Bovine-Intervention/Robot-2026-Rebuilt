package frc.util.led.functions;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.util.Color;

@FunctionalInterface
public interface InterpolationFunction extends Interpolator<Color> {
	public static final InterpolationFunction step = (s, e, t) -> s;
	public static final InterpolationFunction linear = (s, e, t) -> new Color(
		s.red * (1-t) + e.red * t,
		s.green * (1-t) + e.green * t,
		s.blue * (1-t) + e.blue * t
	);

	public default Gradient gradient(Color... colors) {
		return new Gradient(this, colors);
	}
}

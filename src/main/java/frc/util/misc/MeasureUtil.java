package frc.util.misc;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MeasureUtil {
	@SafeVarargs
	@SuppressWarnings("unchecked")
	public static <U extends Unit> Measure<U> average(Measure<U>... a) {
		return (Measure<U>)a[0].unit().ofBaseUnits(Arrays.stream(a).mapToDouble((measure) -> measure.baseUnitMagnitude()).average().orElse(0));
	}

	public static <U extends Unit> boolean isNear(Measure<U> expected, Measure<U> actual, Measure<U> tolerance) {
		return MathUtil.isNear(expected.baseUnitMagnitude(), actual.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
	}

	public static <U extends Unit> boolean isWithin(Measure<U> value, Measure<U> min, Measure<U> max) {
		return MathExtraUtil.isWithin(value.baseUnitMagnitude(), min.baseUnitMagnitude(), max.baseUnitMagnitude());
	}

	@SuppressWarnings("unchecked")
	public static <U extends Unit> Measure<U> clamp(Measure<U> value, Measure<U> low, Measure<U> high) {
		return (Measure<U>)value.unit().ofBaseUnits(MathUtil.clamp(value.baseUnitMagnitude(), low.baseUnitMagnitude(), high.baseUnitMagnitude()));
	}

	@SuppressWarnings("unchecked")
	public static <U extends Unit> Measure<U> interpolate(Measure<U> start, Measure<U> end, double t) {
		return (Measure<U>)start.unit().ofBaseUnits(MathUtil.interpolate(start.baseUnitMagnitude(), end.baseUnitMagnitude(), t));
	}
	public static <U extends Unit> double inverseInterpolate(Measure<U> start, Measure<U> end, Measure<U> t) {
		return MathUtil.inverseInterpolate(start.baseUnitMagnitude(), end.baseUnitMagnitude(), t.baseUnitMagnitude());
	}
}

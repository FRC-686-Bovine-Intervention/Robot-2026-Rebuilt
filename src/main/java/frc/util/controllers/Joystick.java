package frc.util.controllers;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The X and Y axes should be perpendicular to each other
 */
public class Joystick {
	private final Axis x;
	private final Axis y;

	public Joystick(Axis x, Axis y) {
		this.x = x;
		this.y = y;
	}

	public Joystick(DoubleSupplier x, DoubleSupplier y) {
		this(new Axis(x), new Axis(y));
	}

	public static Joystick fromAngleMagnitude(DoubleSupplier angle, DoubleSupplier magnitude) {
		return new Axis(()->Math.cos(angle.getAsDouble())*magnitude.getAsDouble()).asXwithY(new Axis(()->Math.sin(angle.getAsDouble())*magnitude.getAsDouble()));
	}

	public Axis x() {
		return x;
	}

	public Axis y() {
		return y;
	}

	public Vector<N2> toVector() {
		return VecBuilder.fill(x.getAsDouble(), y.getAsDouble());
	}

	public Joystick multiplyX(DoubleSupplier xCoef) {
		return new Joystick(x.multiply(xCoef), y);
	}
	public Joystick multiplyX(double xCoef) {
		return new Joystick(x.multiply(xCoef), y);
	}
	public Joystick invertX() {
		return new Joystick(x.invert(), y);
	}

	public Joystick multiplyY(DoubleSupplier yCoef) {
		return new Joystick(x, y.multiply(yCoef));
	}
	public Joystick multiplyY(double yCoef) {
		return new Joystick(x, y.multiply(yCoef));
	}
	public Joystick invertY() {
		return new Joystick(x, y.invert());
	}

	public Joystick multiply(DoubleSupplier xCoef, DoubleSupplier yCoef) {
		return new Joystick(x.multiply(xCoef), y.multiply(yCoef));
	}
	public Joystick multiply(double xCoef, double yCoef) {
		return new Joystick(x.multiply(xCoef), y.multiply(yCoef));
	}

	public double magnitude() {
		return Math.hypot(x.getAsDouble(), y.getAsDouble());
	}

	public double radsFromPosXCCW() {
		return Math.atan2(y.getAsDouble(), x.getAsDouble());
	}
	public double radsFromPosYCCW() {
		return Math.atan2(-x.getAsDouble(), y.getAsDouble());
	}
	public double radsFromNegXCCW() {
		return Math.atan2(-y.getAsDouble(), -x.getAsDouble());
	}
	public double radsFromNegYCCW() {
		return Math.atan2(x.getAsDouble(), -y.getAsDouble());
	}

	public double radsFromPosXCW() {
		return -radsFromPosXCCW();
	}
	public double radsFromPosYCW() {
		return -radsFromPosYCCW();
	}
	public double radsFromNegXCW() {
		return -radsFromNegXCCW();
	}
	public double radsFromNegYCW() {
		return -radsFromNegYCCW();
	}

	public Joystick roughRadialDeadband(double deadband) {
		return fromMagnitude(() ->
			(magnitude() >= deadband) ? (
				magnitude()
			) : (
				0
			)
		);
	}

	private Joystick fromMagnitude(DoubleSupplier mag) {
		return fromAngleMagnitude(this::radsFromPosXCCW, mag);
	}

	public Joystick smoothRadialDeadband(double deadband) {
		return fromMagnitude(() -> MathUtil.applyDeadband(magnitude(), deadband));
	}

	public Joystick radialSlewRateLimit(double slewRatePerSec) {
		SlewRateLimiter slewRateLimiter = new SlewRateLimiter(slewRatePerSec);
		return fromMagnitude(() -> slewRateLimiter.calculate(magnitude()));
	}

	// private double previousAngle;
	// public Joystick radialSlewRateLimit(double slewRatePerSec) {
	//     SlewRateLimiter slewRateLimiter = new SlewRateLimiter(slewRatePerSec);
	//     if (magnitude() > 0.1) previousAngle = radsFromNegXCCW();
	//     return fromAngleMagnitude(() -> previousAngle, () -> slewRateLimiter.calculate(magnitude()));
	// }

	private static DoubleUnaryOperator sensitivityFunction(double sensitivityVal) {
		return (x) -> sensitivityVal * x * x * x - sensitivityVal * x + x;
	}

	public Joystick radialSensitivity(double sensitivityVal) {
		return fromMagnitude(() -> sensitivityFunction(sensitivityVal).applyAsDouble(magnitude()));
	}

	public Joystick individualSensitivity(double sensitivityVal) {
		var sensFunc = sensitivityFunction(sensitivityVal);
		return new Joystick(() -> sensFunc.applyAsDouble(x.getAsDouble()), () -> sensFunc.applyAsDouble(y.getAsDouble()));
	}

	public static class Axis implements DoubleSupplier {
		private final DoubleSupplier source;

		public Axis(DoubleSupplier source) {
			this.source = source;
		}

		@Override
		public double getAsDouble() {
			return source.getAsDouble();
		}

		public Axis multiply(DoubleSupplier coef) {
			return new Axis(() -> getAsDouble() * coef.getAsDouble());
		}

		public Axis multiply(double coef) {
			return multiply(() -> coef);
		}

		public Axis invert() {
			return multiply(-1);
		}

		public Axis add(DoubleSupplier other) {
			return new Axis(() -> getAsDouble() + other.getAsDouble());
		}

		public Axis add(double other) {
			return add(() -> other);
		}

		public Axis roughDeadband(double deadband) {
			return new Axis(() -> Math.abs(getAsDouble()) < deadband ? 0 : getAsDouble());
		}

		public Axis smoothDeadband(double deadband) {
			return new Axis(() -> MathUtil.applyDeadband(getAsDouble(), deadband));
		}

		public Axis abs() {
			return new Axis(() -> Math.abs(getAsDouble()));
		}

		public Axis sensitivity(double sensitivityVal) {
			return new Axis(() -> sensitivityFunction(sensitivityVal).applyAsDouble(getAsDouble()));
		}

		public Trigger aboveThreshold(double threshold) {
			return aboveThreshold(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
		}

		public Trigger aboveThreshold(double threshold, EventLoop loop) {
			return new Trigger(loop, () -> getAsDouble() >= threshold);
		}

		public Joystick asXwithY(Axis yAxis) {
			return new Joystick(this, yAxis);
		}

		public Joystick asYwithX(Axis xAxis) {
			return new Joystick(xAxis, this);
		}
	}
}

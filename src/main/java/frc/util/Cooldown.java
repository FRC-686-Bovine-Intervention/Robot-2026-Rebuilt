package frc.util;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Cooldown {
	private final Timer timer = new Timer();
	private final double defaultCooldown;
	private double cooldown = 0;

	public Cooldown() {
		this(0);
	}

	public Cooldown(double defaultCooldown) {
		this.defaultCooldown = defaultCooldown;
	}

	public boolean hasExpired() {
		var ret = timer.hasElapsed(cooldown);
		if (ret) {
			timer.stop();
		}
		return ret;
	}

	public double secondsRemaining() {
		return cooldown - timer.get();
	}

	public void reset() {
		reset(defaultCooldown);
	}

	public void reset(double cooldown) {
		this.cooldown = cooldown;
		timer.restart();
	}

	public static <U extends Unit> DoubleSupplier incrementingStepper(String tuningKey, String loggingKey, Time defaultStepTime, Measure<U> defaultStartValue, Measure<U> defaultStepAmount, U outputUnit, BooleanSupplier increment, BooleanSupplier decrement) {
		return new DoubleSupplier() {
			@SuppressWarnings("unchecked")
			private final LoggedTunable<Measure<U>> stepAmount = LoggedTunable.from(tuningKey + "/Step Amount", (value) -> (Measure<U>) defaultStepAmount.unit().of(value), defaultStepAmount.magnitude());
			private final LoggedTunable<Time> stepTimeSecs = LoggedTunable.from(tuningKey + "/Step Time", Seconds::of, defaultStepTime.in(Seconds));

			private final Timer timer = new Timer();

			private double inner = defaultStartValue.in(outputUnit);

			@Override
			public double getAsDouble() {
				var inc = increment.getAsBoolean();
				var dec = decrement.getAsBoolean();
				if (inc || dec) {
					this.timer.start();
				} else {
					this.timer.stop();
					this.timer.reset();
				}
				if (this.timer.advanceIfElapsed(this.stepTimeSecs.get().in(Seconds))) {
					if (inc) {
						this.inner += this.stepAmount.get().in(outputUnit);
					}
					if (dec) {
						this.inner -= this.stepAmount.get().in(outputUnit);
					}
				}
				Logger.recordOutput(loggingKey + "/Target", this.inner, outputUnit);
				return inner;
			}
		};
	}
}

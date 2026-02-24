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
			private final LoggedTunable<Time> stepTime = LoggedTunable.from(tuningKey + "/Step Time", Seconds::of, defaultStepTime.in(Seconds));

			private final Timer timer = new Timer();

			private double inner = defaultStartValue.in(outputUnit);

			@Override
			public double getAsDouble() {
				var inc = increment.getAsBoolean();
				var dec = decrement.getAsBoolean();

				var stepTimeSecs = this.stepTime.get().in(Seconds);

				var incAmount = 0;
				if (!this.timer.isRunning()) {
					if (inc) {
						incAmount += this.stepAmount.get().in(outputUnit);
					}
					if (dec) {
						incAmount -= this.stepAmount.get().in(outputUnit);
					}
				} else if (this.timer.hasElapsed(stepTimeSecs)) {
					this.timer.stop();
					if (inc || dec) {
						this.timer.advanceIfElapsed(stepTimeSecs);
						if (inc) {
							incAmount += this.stepAmount.get().in(outputUnit);
						}
						if (dec) {
							incAmount -= this.stepAmount.get().in(outputUnit);
						}
					} else {
						this.timer.reset();
					}
				}

				if (incAmount != 0.0) {
					this.inner += incAmount;
					this.timer.start();
				}

				Logger.recordOutput(loggingKey + "/Target", this.inner);
				return inner;
			}
		};
	}
}

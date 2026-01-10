package frc.robot.subsystems.drive;

import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.loggerUtil.tunables.Tunable;

public record TiltAccelerationLimits(
	double forwardLimit,
	double backwardLimit,
	double leftLimit,
	double rightLimit
) implements Tunable<TiltAccelerationLimits> {
	public double getMaxTiltAccelerationMPSS(double heading) {
		if (heading < Math.atan2(-this.rightLimit(), -this.backwardLimit())) {
			return -this.backwardLimit() / Math.cos(heading);
		} else if (heading < Math.atan2(-this.rightLimit(), this.forwardLimit())) {
			return -this.rightLimit() / Math.sin(heading);
		} else if (heading < Math.atan2(this.leftLimit(), this.forwardLimit())) {
			return this.forwardLimit() / Math.cos(heading);
		} else if (heading < Math.atan2(this.leftLimit(), -this.backwardLimit())) {
			return this.leftLimit() / Math.sin(heading);
		} else {
			return -this.backwardLimit() / Math.cos(heading);
		}
	}

	@Override
	public LoggedTunable<TiltAccelerationLimits> makeTunable(String key) {
		final var defaultValue = this;
		return new LoggedTunable<>() {
			private final LoggedTunableNumber forwardLimit = LoggedTunable.from(key + "/Forward", defaultValue.forwardLimit());
			private final LoggedTunableNumber backwardLimit = LoggedTunable.from(key + "/Backward", defaultValue.backwardLimit());
			private final LoggedTunableNumber leftLimit = LoggedTunable.from(key + "/Left", defaultValue.leftLimit());
			private final LoggedTunableNumber rightLimit = LoggedTunable.from(key + "/Right", defaultValue.rightLimit());

			private TiltAccelerationLimits cache = defaultValue;

			@Override
			public TiltAccelerationLimits get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new TiltAccelerationLimits(
						this.forwardLimit.get(),
						this.backwardLimit.get(),
						this.leftLimit.get(),
						this.rightLimit.get()
					);
				}
				return this.cache;
			}

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.forwardLimit, this.backwardLimit, this.leftLimit, this.rightLimit);
			}
		};
	}
}

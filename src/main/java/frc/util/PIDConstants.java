package frc.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.loggerUtil.tunables.Tunable;

public record PIDConstants(double kP, double kI, double kD) implements Tunable<PIDConstants> {
	@Override
	public LoggedTunable<PIDConstants> makeTunable(String key) {
		final var defaultValue = this;
		return new LoggedTunable<>() {
			private final LoggedTunableNumber kP = LoggedTunable.from(key + "/kP", defaultValue.kP());
			private final LoggedTunableNumber kI = LoggedTunable.from(key + "/kI", defaultValue.kI());
			private final LoggedTunableNumber kD = LoggedTunable.from(key + "/kD", defaultValue.kD());

			private PIDConstants cache = defaultValue;

			@Override
			public PIDConstants get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new PIDConstants(
						this.kP.getAsDouble(),
						this.kI.getAsDouble(),
						this.kD.getAsDouble()
					);
				}
				return this.cache;
			}

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.kP, this.kI, this.kD);
			}
		};
	}

	public void update(PIDController pid) {
		pid.setPID(
			this.kP(),
			this.kI(),
			this.kD()
		);
	}
	public void update(ProfiledPIDController pid) {
		pid.setPID(
			this.kP(),
			this.kI(),
			this.kD()
		);
	}
	public void update(SlotConfigs pid) {
		pid
			.withKP(this.kP())
			.withKI(this.kI())
			.withKD(this.kD())
		;
	}
	public void update(Slot0Configs pid) {
		pid
			.withKP(this.kP())
			.withKI(this.kI())
			.withKD(this.kD())
		;
	}
	public void update(Slot1Configs pid) {
		pid
			.withKP(this.kP())
			.withKI(this.kI())
			.withKD(this.kD())
		;
	}
	public void update(Slot2Configs pid) {
		pid
			.withKP(this.kP())
			.withKI(this.kI())
			.withKD(this.kD())
		;
	}
}

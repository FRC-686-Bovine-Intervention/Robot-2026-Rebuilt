package frc.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.loggerUtil.tunables.Tunable;

public record FFConstants(double kS, double kG, double kV, double kA) implements Tunable<FFConstants> {
	@Override
	public LoggedTunable<FFConstants> makeTunable(String key) {
		final var defaultValue = this;
		return new LoggedTunable<>() {
			private final LoggedTunableNumber kS = LoggedTunable.from(key + "/kS", defaultValue.kS());
			private final LoggedTunableNumber kG = LoggedTunable.from(key + "/kG", defaultValue.kG());
			private final LoggedTunableNumber kV = LoggedTunable.from(key + "/kV", defaultValue.kV());
			private final LoggedTunableNumber kA = LoggedTunable.from(key + "/kA", defaultValue.kA());

			private FFConstants cache = defaultValue;

			@Override
			public FFConstants get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new FFConstants(
						this.kS.getAsDouble(),
						this.kG.getAsDouble(),
						this.kV.getAsDouble(),
						this.kA.getAsDouble()
					);
				}
				return this.cache;
			}

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.kS, this.kG, this.kV, this.kA);
			}
		};
	}


	public void update(SimpleMotorFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
	}
	public void update(ArmFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKg(this.kG());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
	}
	public void update(ElevatorFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKg(this.kG());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
	}

	public void update(SlotConfigs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
	}
	public void update(Slot0Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
	}
	public void update(Slot1Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
	}
	public void update(Slot2Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
	}
}

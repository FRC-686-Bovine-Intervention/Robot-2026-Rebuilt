package frc.util;

import java.util.function.DoubleUnaryOperator;

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

public record FFGains(double kS, double kG, double kV, double kA) implements Tunable<FFGains> {
	@Override
	public LoggedTunable<FFGains> makeTunable(String key) {
		final var defaultValue = this;
		return new LoggedTunable<>() {
			private final LoggedTunableNumber kS = LoggedTunable.from(key + "/kS", defaultValue.kS());
			private final LoggedTunableNumber kG = LoggedTunable.from(key + "/kG", defaultValue.kG());
			private final LoggedTunableNumber kV = LoggedTunable.from(key + "/kV", defaultValue.kV());
			private final LoggedTunableNumber kA = LoggedTunable.from(key + "/kA", defaultValue.kA());

			private FFGains cache = defaultValue;

			@Override
			public FFGains get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new FFGains(
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

	public SimpleMotorFeedforward update(SimpleMotorFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
		return ff;
	}

	public ArmFeedforward update(ArmFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKg(this.kG());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
		return ff;
	}

	public ElevatorFeedforward update(ElevatorFeedforward ff) {
		ff.setKs(this.kS());
		ff.setKg(this.kG());
		ff.setKv(this.kV());
		ff.setKa(this.kA());
		return ff;
	}

	public SlotConfigs update(SlotConfigs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
		return ff;
	}

	public Slot0Configs update(Slot0Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
		return ff;
	}

	public Slot1Configs update(Slot1Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
		return ff;
	}

	public Slot2Configs update(Slot2Configs ff) {
		ff
			.withKS(this.kS())
			.withKG(this.kG())
			.withKV(this.kV())
			.withKA(this.kA())
		;
		return ff;
	}

	public FFGains mul(double mul) {
		return new FFGains(
			this.kS() * mul,
			this.kG() * mul,
			this.kV() * mul,
			this.kA() * mul
		);
	}

	public FFGains map(DoubleUnaryOperator mappingFunction) {
		return new FFGains(
			mappingFunction.applyAsDouble(this.kS()),
			mappingFunction.applyAsDouble(this.kG()),
			mappingFunction.applyAsDouble(this.kV()),
			mappingFunction.applyAsDouble(this.kA())
		);
	}
}

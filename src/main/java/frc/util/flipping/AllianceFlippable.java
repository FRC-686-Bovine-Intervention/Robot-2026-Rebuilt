package frc.util.flipping;

import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

public interface AllianceFlippable<T> {
	public T flip(FieldFlipType flipType);
	public default T flip() {
		return this.flip(AllianceFlipUtil.defaultFlipType);
	}
}

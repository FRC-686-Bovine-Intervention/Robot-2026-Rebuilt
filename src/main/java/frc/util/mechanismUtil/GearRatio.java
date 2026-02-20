package frc.util.mechanismUtil;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class GearRatio {
	public static double planetaryReduction(int sunCount, int ringCount) {
		return (double)(sunCount + ringCount) / sunCount;
	}
	public static final double ULTRAPLANETARY_3_1 = GearRatio.planetaryReduction(29, 55); // 84:29 reduction
	public static final double ULTRAPLANETARY_4_1 = GearRatio.planetaryReduction(21, 55); // 76:21 reduction
	public static final double ULTRAPLANETARY_5_1 = GearRatio.planetaryReduction(13, 55); // 68:13 reduction

	private final double reduction;
	private final GearRatio inverse;

	private GearRatio(double reduction, GearRatio inverse) {
		this.reduction = reduction;
		this.inverse = inverse;
	}
	public GearRatio(double reduction) {
		this.reduction = reduction;
		this.inverse = new GearRatio(1.0 / reduction, this);
	}
	public GearRatio() {
		this(1);
	}

	public double reductionSigned() {
		return this.reduction;
	}
	public double reductionUnsigned() {
		return Math.abs(this.reductionSigned());
	}
	public GearRatio inverse() {
		return this.inverse;
	}

	public Gear gear(double teeth) {
		return new Gear(teeth, this);
	}
	public Sprocket sprocket(double teeth) {
		return new Sprocket(teeth, this);
	}
	public GearRatio planetary(double reduction) {
		return new GearRatio(this.reductionSigned() * reduction);
	}
	public GearRatio then(GearRatio other) {
		return new GearRatio(this.reductionSigned() * other.reductionSigned());
	}
	public GearRatio unsigned() {
		return new GearRatio(Math.abs(this.reductionSigned()));
	}

	public double applySigned(double a) {
		return a * this.inverse().reductionSigned();
	}
	public <U extends Unit> Measure<U> applySigned(Measure<U> angle) {
		return angle.times(this.inverse().reductionSigned());
	}
	public Angle applySigned(Angle angle) {
		return angle.times(this.inverse().reductionSigned());
	}
	public AngularVelocity applySigned(AngularVelocity angle) {
		return angle.times(this.inverse().reductionSigned());
	}
	public AngularAcceleration applySigned(AngularAcceleration angle) {
		return angle.times(this.inverse().reductionSigned());
	}

	public double applyUnsigned(double a) {
		return a * this.inverse().reductionUnsigned();
	}
	public <U extends Unit> Measure<U> applyUnsigned(Measure<U> angle) {
		return angle.times(this.inverse().reductionUnsigned());
	}
	public Angle applyUnsigned(Angle angle) {
		return angle.times(this.inverse().reductionUnsigned());
	}
	public AngularVelocity applyUnsigned(AngularVelocity angle) {
		return angle.times(this.inverse().reductionUnsigned());
	}
	public AngularAcceleration applyUnsigned(AngularAcceleration angle) {
		return angle.times(this.inverse().reductionUnsigned());
	}

	public static class Gear {
		private final double teeth;
		private final GearRatio axle;

		private Gear(double teeth, GearRatio axle) {
			this.teeth = teeth;
			this.axle = axle;
		}

		public GearRatio axle() {
			return axle;
		}

		public Gear gear(double teeth) {
			return new Gear(teeth, new GearRatio(this.axle().reductionSigned() * -teeth / this.teeth));
		}
	}
	public static class Sprocket {
		private final double teeth;
		private final GearRatio axle;

		private Sprocket(double teeth, GearRatio axle) {
			this.teeth = teeth;
			this.axle = axle;
		}

		public GearRatio axle() {
			return axle;
		}

		public GearRatio sprocket(double teeth) {
			return new GearRatio(this.axle().reductionSigned() * teeth / this.teeth);
		}

		public LinearRelation chain(Measure<DistanceUnit> linkSize) {
			return LinearRelation.wheelCircumference(linkSize.times(teeth));
		}
	}
}

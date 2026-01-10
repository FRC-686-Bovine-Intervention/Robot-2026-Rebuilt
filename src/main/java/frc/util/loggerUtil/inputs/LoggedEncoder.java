package frc.util.loggerUtil.inputs;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class LoggedEncoder implements StructSerializable {
	private double positionRads = 0.0;
	private double velocityRadsPerSec = 0.0;

	public LoggedEncoder() {}

	private LoggedEncoder(double positionRads, double velocityRadsPerSec) {
		this.positionRads = positionRads;
		this.velocityRadsPerSec = velocityRadsPerSec;
	}

	/**
	 * @return The current position of the encoder in {@link #edu.wpi.first.units.Units.Radians radians}
	 */
	public double getPositionRads() {
		return this.positionRads;
	}
	/**
	 * @return The current velocity of the encoder in {@link #edu.wpi.first.units.Units.RadiansPerSecond radians per second}
	 */
	public double getVelocityRadsPerSec() {
		return this.velocityRadsPerSec;
	}

	public void setPositionRads(double positionRads) {
		this.positionRads = positionRads;
	}

	public void setVelocityRadsPerSec(double velocityRadsPerSec) {
		this.velocityRadsPerSec = velocityRadsPerSec;
	}

	public static record EncoderStatusSignalCache(
		StatusSignal<Angle> position,
		StatusSignal<AngularVelocity> velocity
	) {
		public static EncoderStatusSignalCache from(TalonFX talonFX) {
			return new EncoderStatusSignalCache(talonFX.getRotorPosition(), talonFX.getRotorVelocity());
		}
		public static EncoderStatusSignalCache from(TalonFXS talonFXS) {
			return new EncoderStatusSignalCache(talonFXS.getRotorPosition(), talonFXS.getRotorVelocity());
		}
		public static EncoderStatusSignalCache from(CANcoder cancoder) {
			return new EncoderStatusSignalCache(cancoder.getPosition(), cancoder.getVelocity());
		}

		public BaseStatusSignal[] getStatusSignals() {
			return new BaseStatusSignal[] {
				this.position(),
				this.velocity(),
			};
		}
	}

	public void updateFrom(EncoderStatusSignalCache statusSignals) {
		this.setPositionRads(Units.rotationsToRadians(statusSignals.position().getValueAsDouble()));
		this.setVelocityRadsPerSec(Units.rotationsToRadians(statusSignals.velocity().getValueAsDouble()));
	}

	public void updateFrom(RelativeEncoder encoder) {
		this.setPositionRads(Units.rotationsToRadians(encoder.getPosition()));
		this.setVelocityRadsPerSec(Units.rotationsToRadians(encoder.getVelocity()));
	}
	public void updateFrom(AbsoluteEncoder encoder) {
		this.setPositionRads(Units.rotationsToRadians(encoder.getPosition()));
		this.setVelocityRadsPerSec(Units.rotationsToRadians(encoder.getVelocity()));
	}

	public static final LoggedEncoderStruct struct = new LoggedEncoderStruct();
	public static class LoggedEncoderStruct implements Struct<LoggedEncoder> {
		@Override
		public Class<LoggedEncoder> getTypeClass() {
			return LoggedEncoder.class;
		}

		@Override
		public String getTypeName() {
			return "Encoder";
		}

		@Override
		public int getSize() {
			return kSizeDouble * 2;
		}

		@Override
		public String getSchema() {
			return "double PositionRad;double VelocityRadPerSec";
		}

		@Override
		public LoggedEncoder unpack(ByteBuffer bb) {
			var positionRads = bb.getDouble();
			var velocityRadPerSec = bb.getDouble();
			return new LoggedEncoder(positionRads, velocityRadPerSec);
		}

		@Override
		public void unpackInto(LoggedEncoder out, ByteBuffer bb) {
			out.setPositionRads(bb.getDouble());
			out.setVelocityRadsPerSec(bb.getDouble());
		}

		@Override
		public void pack(ByteBuffer bb, LoggedEncoder value) {
			bb.putDouble(value.getPositionRads());
			bb.putDouble(value.getVelocityRadsPerSec());
		}
	}
}

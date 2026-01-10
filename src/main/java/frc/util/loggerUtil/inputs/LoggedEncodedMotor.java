package frc.util.loggerUtil.inputs;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.util.loggerUtil.inputs.LoggedEncoder.EncoderStatusSignalCache;
import frc.util.loggerUtil.inputs.LoggedMotor.MotorStatusSignalCache;

public class LoggedEncodedMotor implements StructSerializable {
	public final LoggedEncoder encoder;
	public final LoggedMotor motor;

	public LoggedEncodedMotor() {
		this(new LoggedEncoder(), new LoggedMotor());
	}
	public LoggedEncodedMotor(LoggedEncoder encoder, LoggedMotor motor) {
		this.encoder = encoder;
		this.motor = motor;
	}

	public static record EncodedMotorStatusSignalCache(
		EncoderStatusSignalCache encoder,
		MotorStatusSignalCache motor
	) {
		public static EncodedMotorStatusSignalCache from(TalonFX talonFX) {
			return new EncodedMotorStatusSignalCache(EncoderStatusSignalCache.from(talonFX), MotorStatusSignalCache.from(talonFX));
		}
		public static EncodedMotorStatusSignalCache from(TalonFXS talonFXS) {
			return new EncodedMotorStatusSignalCache(EncoderStatusSignalCache.from(talonFXS), MotorStatusSignalCache.from(talonFXS));
		}
	}

	public void updateFrom(EncodedMotorStatusSignalCache statusSignals) {
		this.encoder.updateFrom(statusSignals.encoder());
		this.motor.updateFrom(statusSignals.motor());
	}

	public void updateFrom(SparkMax spark) {
		this.encoder.updateFrom(spark.getEncoder());
		this.motor.updateFrom(spark);
	}

	public static final LoggedEncodedMotorStruct struct = new LoggedEncodedMotorStruct();
	public static class LoggedEncodedMotorStruct implements Struct<LoggedEncodedMotor> {
		@Override
		public Class<LoggedEncodedMotor> getTypeClass() {
			return LoggedEncodedMotor.class;
		}

		@Override
		public String getTypeName() {
			return "EncodedMotor";
		}

		@Override
		public Struct<?>[] getNested() {
			return new Struct<?>[] {LoggedEncoder.struct, LoggedMotor.struct};
		}

		@Override
		public int getSize() {
			return LoggedEncoder.struct.getSize() * 1 + LoggedMotor.struct.getSize() * 1;
		}

		@Override
		public String getSchema() {
			return "Encoder encoder; Motor motor";
		}

		@Override
		public LoggedEncodedMotor unpack(ByteBuffer bb) {
			var encoder = LoggedEncoder.struct.unpack(bb);
			var motor = LoggedMotor.struct.unpack(bb);
			return new LoggedEncodedMotor(encoder, motor);
		}

		@Override
		public void unpackInto(LoggedEncodedMotor out, ByteBuffer bb) {
			LoggedEncoder.struct.unpackInto(out.encoder, bb);
			LoggedMotor.struct.unpackInto(out.motor, bb);
		}

		@Override
		public void pack(ByteBuffer bb, LoggedEncodedMotor value) {
			LoggedEncoder.struct.pack(bb, value.encoder);
			LoggedMotor.struct.pack(bb, value.motor);
		}
	}
}

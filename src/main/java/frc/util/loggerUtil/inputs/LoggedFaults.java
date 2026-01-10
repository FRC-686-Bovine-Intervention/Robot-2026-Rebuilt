package frc.util.loggerUtil.inputs;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;

public class LoggedFaults implements StructSerializable {
	public final DeviceFaults activeFaults = new DeviceFaults();
	public final DeviceFaults stickyFaults = new DeviceFaults();

	public void updateFrom(TalonFX talonFX) {
		var activeBitfield = 0x0000000000000000L;
		var stickyBitfield = 0x0000000000000000L;
		for (var faultType : FaultType.possibleTalonFXFaults) {
			if (faultType.getFaultFrom(talonFX)) {
				activeBitfield |= faultType.getBitmask();
			}
			if (faultType.getStickyFaultFrom(talonFX)) {
				stickyBitfield |= faultType.getBitmask();
			}
		}
		this.activeFaults.mut_setBitfield(activeBitfield);
		this.stickyFaults.mut_setBitfield(stickyBitfield);
	}

	public void updateFrom(TalonFXS talonFXS) {
		var activeBitfield = 0x0000000000000000L;
		var stickyBitfield = 0x0000000000000000L;
		for (var faultType : FaultType.possibleTalonFXSFaults) {
			if (faultType.getFaultFrom(talonFXS)) {
				activeBitfield |= faultType.getBitmask();
			}
			if (faultType.getStickyFaultFrom(talonFXS)) {
				stickyBitfield |= faultType.getBitmask();
			}
		}
		this.activeFaults.mut_setBitfield(activeBitfield);
		this.stickyFaults.mut_setBitfield(stickyBitfield);
	}

	public void updateFrom(CANcoder cancoder) {
		var activeBitfield = 0x0000000000000000L;
		var stickyBitfield = 0x0000000000000000L;
		for (var faultType : FaultType.possibleCancoderFaults) {
			if (faultType.getFaultFrom(cancoder)) {
				activeBitfield |= faultType.getBitmask();
			}
			if (faultType.getStickyFaultFrom(cancoder)) {
				stickyBitfield |= faultType.getBitmask();
			}
		}
		this.activeFaults.mut_setBitfield(activeBitfield);
		this.stickyFaults.mut_setBitfield(stickyBitfield);
	}

	public void updateFrom(SparkMax sparkMax) {
		var activeBitfield = 0x0000000000000000L;
		var stickyBitfield = 0x0000000000000000L;
		for (var faultType : FaultType.possibleSparkMaxFaults) {
			if (faultType.getFaultFrom(sparkMax)) {
				activeBitfield |= faultType.getBitmask();
			}
			if (faultType.getStickyFaultFrom(sparkMax)) {
				stickyBitfield |= faultType.getBitmask();
			}
		}
		this.activeFaults.mut_setBitfield(activeBitfield);
		this.stickyFaults.mut_setBitfield(stickyBitfield);
	}

	public static final LoggedFaultsStruct struct = new LoggedFaultsStruct();
	public static class LoggedFaultsStruct implements Struct<LoggedFaults> {
		@Override
		public Class<LoggedFaults> getTypeClass() {
			return LoggedFaults.class;
		}

		@Override
		public String getTypeName() {
			return "Faults";
		}

		@Override
		public int getSize() {
			return DeviceFaults.struct.getSize() * 2;
		}

		@Override
		public String getSchema() {
			return "DeviceFaults activeFaults; DeviceFaults stickyFaults";
		}

		@Override
		public LoggedFaults unpack(ByteBuffer bb) {
			var faults = new LoggedFaults();
			this.unpackInto(faults, bb);
			return faults;
		}

		@Override
		public void unpackInto(LoggedFaults out, ByteBuffer bb) {
			DeviceFaults.struct.unpackInto(out.activeFaults, bb);
			DeviceFaults.struct.unpackInto(out.stickyFaults, bb);
		}

		@Override
		public void pack(ByteBuffer bb, LoggedFaults value) {
			DeviceFaults.struct.pack(bb, value.activeFaults);
			DeviceFaults.struct.pack(bb, value.stickyFaults);
		}
	}
}

package frc.util.faults;

import java.nio.ByteBuffer;
import java.util.Arrays;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class DeviceFaults implements StructSerializable {
	public static final long noneMask = 0x0000000000000000L;
	public static final long allMask = 0xFFFFFFFFFFFFFFFFL;

	private long bitfield;

	public DeviceFaults() {
		this(noneMask);
	}

	public DeviceFaults(long bitfield) {
		this.bitfield = bitfield;
	}

	public long getRawBitfield() {
		return this.bitfield;
	}
	public void mut_setBitfield(long bitfield) {
		this.bitfield = bitfield;
	}

	public boolean isAllOk() {
		return this.getRawBitfield() == noneMask;
	}
	public boolean getFault(FaultType faultType) {
		return (this.getRawBitfield() & faultType.getBitmask()) == faultType.getBitmask();
	}
	public boolean anyOf(long bitmask) {
		return (this.getRawBitfield() & bitmask) != noneMask;
	}

	public FaultType[] getActiveFaults() {
		return Arrays
			.stream(FaultType.values())
			.filter(this::getFault)
			.toArray(FaultType[]::new)
		;
	}
	public FaultType[] getActiveFaults(long bitmask) {
		return Arrays
			.stream(FaultType.values())
			.filter((faultType) -> (this.getRawBitfield() & faultType.getBitmask() & bitmask) == faultType.getBitmask())
			.toArray(FaultType[]::new)
		;
	}

	public static final DeviceFaultsStruct struct = new DeviceFaultsStruct();
	public static class DeviceFaultsStruct implements Struct<DeviceFaults> {
		@Override
		public Class<DeviceFaults> getTypeClass() {
			return DeviceFaults.class;
		}

		@Override
		public String getTypeName() {
			return "DeviceFaults";
		}

		@Override
		public int getSize() {
			return kSizeInt64 * 1;
		}

		@Override
		public String getSchema() {
			return "long bitfield";
		}

		@Override
		public DeviceFaults unpack(ByteBuffer bb) {
			var bitfield = bb.getLong();
			return new DeviceFaults(bitfield);
		}

		@Override
		public void unpackInto(DeviceFaults out, ByteBuffer bb) {
			out.mut_setBitfield(bb.getLong());
		}

		@Override
		public void pack(ByteBuffer bb, DeviceFaults value) {
			bb.putLong(value.bitfield);
		}
	}

	public static enum FaultType {
		Hardware(0),
		BootDuringEnable(1),
		UsingUnlicensedFeature(2),
		ForwardHardLimit(3),
		ReverseHardLimit(4),
		ForwardSoftLimit(5),
		ReverseSoftLimit(6),
		RemoteSensorDataInvalid(7),
		MissingRemoteLimitSwitch(8),
		MissingRemoteSoftLimit(9),
		ProcessorTemperature(10),
		DeviceTemperature(11),
		BridgeBrownout(12),
		Undervoltage(13),
		SupplyOvervoltage(14),
		UnstableSupplyVoltage(15),
		StatorCurrentLimit(16),
		SupplyCurrentLimit(17),
		FusedCancoderOutOfSync(18),
		UsingFusedCancoderWhileUnlicensed(19),
		RemoteSensorPositionOverflow(20),
		RemoteSensorReset(21),
		MissingDifferentialTalonFX(22),
		StaticBrakeDisabled(23),
		BridgeShort(24),
		DriveDisabledHallSensor(25),
		HallSensorMissing(26),
		MotorTempSensorMissing(27),
		MotorTempSensorTooHot(28),
		BadMagnet(29),
		CAN(30),
		EscEepromFault(31),
		EscEepromWarning(32),
		ExtEeprom(33),
		MotorType(34),
		GateDriver(35),
		Overcurrent(36),
		Stall(37),
		Temperature(38),
		SensorFault(39),
		SensorWarning(40),
		HasReset(41),
		UnknownWarning(62),
		UnknownFault(63),
		;
		private final long bitmask;
		FaultType(int bitPos) {
			this.bitmask = 0x0000000000000001L << bitPos;
		}
		public long getBitmask() {
			return this.bitmask;
		}
		public boolean isPartOf(long bitfield) {
			return (bitfield & this.getBitmask()) == this.getBitmask();
		}
		public static final FaultType[] possibleTalonFXFaults = new FaultType[] {
			BootDuringEnable,
			BridgeBrownout,
			DeviceTemperature,
			ForwardHardLimit,
			ForwardSoftLimit,
			FusedCancoderOutOfSync,
			Hardware,
			MissingDifferentialTalonFX,
			MissingRemoteLimitSwitch,
			MissingRemoteSoftLimit,
			ProcessorTemperature,
			RemoteSensorDataInvalid,
			RemoteSensorPositionOverflow,
			RemoteSensorReset,
			ReverseHardLimit,
			ReverseSoftLimit,
			StaticBrakeDisabled,
			StatorCurrentLimit,
			SupplyCurrentLimit,
			SupplyOvervoltage,
			Undervoltage,
			UnstableSupplyVoltage,
			UsingFusedCancoderWhileUnlicensed,
			UsingUnlicensedFeature,
		};
		public static BaseStatusSignal[] getFaultStatusSignals(TalonFX talonFX) {
			return new BaseStatusSignal[] {
				talonFX.getFault_BootDuringEnable(),
				talonFX.getFault_BridgeBrownout(),
				talonFX.getFault_DeviceTemp(),
				talonFX.getFault_ForwardHardLimit(),
				talonFX.getFault_ForwardSoftLimit(),
				talonFX.getFault_FusedSensorOutOfSync(),
				talonFX.getFault_Hardware(),
				talonFX.getFault_MissingDifferentialFX(),
				talonFX.getFault_MissingHardLimitRemote(),
				talonFX.getFault_MissingSoftLimitRemote(),
				talonFX.getFault_ProcTemp(),
				talonFX.getFault_RemoteSensorDataInvalid(),
				talonFX.getFault_RemoteSensorPosOverflow(),
				talonFX.getFault_RemoteSensorReset(),
				talonFX.getFault_ReverseHardLimit(),
				talonFX.getFault_ReverseSoftLimit(),
				talonFX.getFault_StaticBrakeDisabled(),
				talonFX.getFault_StatorCurrLimit(),
				talonFX.getFault_SupplyCurrLimit(),
				talonFX.getFault_OverSupplyV(),
				talonFX.getFault_Undervoltage(),
				talonFX.getFault_UnstableSupplyV(),
				talonFX.getFault_UsingFusedCANcoderWhileUnlicensed(),
				talonFX.getFault_UnlicensedFeatureInUse(),
			};
		}
		public static BaseStatusSignal[] getStickyFaultStatusSignals(TalonFX talonFX) {
			return new BaseStatusSignal[] {
				talonFX.getStickyFault_BootDuringEnable(),
				talonFX.getStickyFault_BridgeBrownout(),
				talonFX.getStickyFault_DeviceTemp(),
				talonFX.getStickyFault_ForwardHardLimit(),
				talonFX.getStickyFault_ForwardSoftLimit(),
				talonFX.getStickyFault_FusedSensorOutOfSync(),
				talonFX.getStickyFault_Hardware(),
				talonFX.getStickyFault_MissingDifferentialFX(),
				talonFX.getStickyFault_MissingHardLimitRemote(),
				talonFX.getStickyFault_MissingSoftLimitRemote(),
				talonFX.getStickyFault_ProcTemp(),
				talonFX.getStickyFault_RemoteSensorDataInvalid(),
				talonFX.getStickyFault_RemoteSensorPosOverflow(),
				talonFX.getStickyFault_RemoteSensorReset(),
				talonFX.getStickyFault_ReverseHardLimit(),
				talonFX.getStickyFault_ReverseSoftLimit(),
				talonFX.getStickyFault_StaticBrakeDisabled(),
				talonFX.getStickyFault_StatorCurrLimit(),
				talonFX.getStickyFault_SupplyCurrLimit(),
				talonFX.getStickyFault_OverSupplyV(),
				talonFX.getStickyFault_Undervoltage(),
				talonFX.getStickyFault_UnstableSupplyV(),
				talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed(),
				talonFX.getStickyFault_UnlicensedFeatureInUse(),
			};
		}
		public boolean getFaultFrom(TalonFX talonFX) {
			return switch (this) {
				case BootDuringEnable -> talonFX.getFault_BootDuringEnable().getValue();
				case BridgeBrownout -> talonFX.getFault_BridgeBrownout().getValue();
				case DeviceTemperature -> talonFX.getFault_DeviceTemp().getValue();
				case ForwardHardLimit -> talonFX.getFault_ForwardHardLimit().getValue();
				case ForwardSoftLimit -> talonFX.getFault_ForwardSoftLimit().getValue();
				case FusedCancoderOutOfSync -> talonFX.getFault_FusedSensorOutOfSync().getValue();
				case Hardware -> talonFX.getFault_Hardware().getValue();
				case MissingDifferentialTalonFX -> talonFX.getFault_MissingDifferentialFX().getValue();
				case MissingRemoteLimitSwitch -> talonFX.getFault_MissingHardLimitRemote().getValue();
				case MissingRemoteSoftLimit -> talonFX.getFault_MissingSoftLimitRemote().getValue();
				case ProcessorTemperature -> talonFX.getFault_ProcTemp().getValue();
				case RemoteSensorDataInvalid -> talonFX.getFault_RemoteSensorDataInvalid().getValue();
				case RemoteSensorPositionOverflow -> talonFX.getFault_RemoteSensorPosOverflow().getValue();
				case RemoteSensorReset -> talonFX.getFault_RemoteSensorReset().getValue();
				case ReverseHardLimit -> talonFX.getFault_ReverseHardLimit().getValue();
				case ReverseSoftLimit -> talonFX.getFault_ReverseSoftLimit().getValue();
				case StaticBrakeDisabled -> talonFX.getFault_StaticBrakeDisabled().getValue();
				case StatorCurrentLimit -> talonFX.getFault_StatorCurrLimit().getValue();
				case SupplyCurrentLimit -> talonFX.getFault_SupplyCurrLimit().getValue();
				case SupplyOvervoltage -> talonFX.getFault_OverSupplyV().getValue();
				case Undervoltage -> talonFX.getFault_Undervoltage().getValue();
				case UnstableSupplyVoltage -> talonFX.getFault_UnstableSupplyV().getValue();
				case UsingFusedCancoderWhileUnlicensed -> talonFX.getFault_UsingFusedCANcoderWhileUnlicensed().getValue();
				case UsingUnlicensedFeature -> talonFX.getFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public boolean getStickyFaultFrom(TalonFX talonFX) {
			return switch (this) {
				case BootDuringEnable -> talonFX.getStickyFault_BootDuringEnable().getValue();
				case BridgeBrownout -> talonFX.getStickyFault_BridgeBrownout().getValue();
				case DeviceTemperature -> talonFX.getStickyFault_DeviceTemp().getValue();
				case ForwardHardLimit -> talonFX.getStickyFault_ForwardHardLimit().getValue();
				case ForwardSoftLimit -> talonFX.getStickyFault_ForwardSoftLimit().getValue();
				case FusedCancoderOutOfSync -> talonFX.getStickyFault_FusedSensorOutOfSync().getValue();
				case Hardware -> talonFX.getStickyFault_Hardware().getValue();
				case MissingDifferentialTalonFX -> talonFX.getStickyFault_MissingDifferentialFX().getValue();
				case MissingRemoteLimitSwitch -> talonFX.getStickyFault_MissingHardLimitRemote().getValue();
				case MissingRemoteSoftLimit -> talonFX.getStickyFault_MissingSoftLimitRemote().getValue();
				case ProcessorTemperature -> talonFX.getStickyFault_ProcTemp().getValue();
				case RemoteSensorDataInvalid -> talonFX.getStickyFault_RemoteSensorDataInvalid().getValue();
				case RemoteSensorPositionOverflow -> talonFX.getStickyFault_RemoteSensorPosOverflow().getValue();
				case RemoteSensorReset -> talonFX.getStickyFault_RemoteSensorReset().getValue();
				case ReverseHardLimit -> talonFX.getStickyFault_ReverseHardLimit().getValue();
				case ReverseSoftLimit -> talonFX.getStickyFault_ReverseSoftLimit().getValue();
				case StaticBrakeDisabled -> talonFX.getStickyFault_StaticBrakeDisabled().getValue();
				case StatorCurrentLimit -> talonFX.getStickyFault_StatorCurrLimit().getValue();
				case SupplyCurrentLimit -> talonFX.getStickyFault_SupplyCurrLimit().getValue();
				case SupplyOvervoltage -> talonFX.getStickyFault_OverSupplyV().getValue();
				case Undervoltage -> talonFX.getStickyFault_Undervoltage().getValue();
				case UnstableSupplyVoltage -> talonFX.getStickyFault_UnstableSupplyV().getValue();
				case UsingFusedCancoderWhileUnlicensed -> talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue();
				case UsingUnlicensedFeature -> talonFX.getStickyFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public StatusCode clearStickyFaultOn(TalonFX talonFX) {
			return switch (this) {
				case BootDuringEnable -> talonFX.clearStickyFault_BootDuringEnable();
				case BridgeBrownout -> talonFX.clearStickyFault_BridgeBrownout();
				case DeviceTemperature -> talonFX.clearStickyFault_DeviceTemp();
				case ForwardHardLimit -> talonFX.clearStickyFault_ForwardHardLimit();
				case ForwardSoftLimit -> talonFX.clearStickyFault_ForwardSoftLimit();
				case FusedCancoderOutOfSync -> talonFX.clearStickyFault_FusedSensorOutOfSync();
				case Hardware -> talonFX.clearStickyFault_Hardware();
				case MissingDifferentialTalonFX -> talonFX.clearStickyFault_MissingDifferentialFX();
				case MissingRemoteLimitSwitch -> talonFX.clearStickyFault_MissingHardLimitRemote();
				case MissingRemoteSoftLimit -> talonFX.clearStickyFault_MissingSoftLimitRemote();
				case ProcessorTemperature -> talonFX.clearStickyFault_ProcTemp();
				case RemoteSensorDataInvalid -> talonFX.clearStickyFault_RemoteSensorDataInvalid();
				case RemoteSensorPositionOverflow -> talonFX.clearStickyFault_RemoteSensorPosOverflow();
				case RemoteSensorReset -> talonFX.clearStickyFault_RemoteSensorReset();
				case ReverseHardLimit -> talonFX.clearStickyFault_ReverseHardLimit();
				case ReverseSoftLimit -> talonFX.clearStickyFault_ReverseSoftLimit();
				case StaticBrakeDisabled -> talonFX.clearStickyFault_StaticBrakeDisabled();
				case StatorCurrentLimit -> talonFX.clearStickyFault_StatorCurrLimit();
				case SupplyCurrentLimit -> talonFX.clearStickyFault_SupplyCurrLimit();
				case SupplyOvervoltage -> talonFX.clearStickyFault_OverSupplyV();
				case Undervoltage -> talonFX.clearStickyFault_Undervoltage();
				case UnstableSupplyVoltage -> talonFX.clearStickyFault_UnstableSupplyV();
				case UsingFusedCancoderWhileUnlicensed -> talonFX.clearStickyFault_UsingFusedCANcoderWhileUnlicensed();
				case UsingUnlicensedFeature -> talonFX.clearStickyFault_UnlicensedFeatureInUse();
				default -> StatusCode.OK;
			};
		}
		public static final FaultType[] possibleTalonFXSFaults = new FaultType[] {
			BootDuringEnable,
			BridgeShort,
			BridgeBrownout,
			DeviceTemperature,
			DriveDisabledHallSensor,
			ForwardHardLimit,
			ForwardSoftLimit,
			FusedCancoderOutOfSync,
			HallSensorMissing,
			Hardware,
			MissingDifferentialTalonFX,
			MissingRemoteLimitSwitch,
			MissingRemoteSoftLimit,
			MotorTempSensorMissing,
			MotorTempSensorTooHot,
			ProcessorTemperature,
			RemoteSensorDataInvalid,
			RemoteSensorPositionOverflow,
			RemoteSensorReset,
			ReverseHardLimit,
			ReverseSoftLimit,
			StaticBrakeDisabled,
			StatorCurrentLimit,
			SupplyCurrentLimit,
			SupplyOvervoltage,
			Undervoltage,
			UnstableSupplyVoltage,
			UsingFusedCancoderWhileUnlicensed,
			UsingUnlicensedFeature,
		};
		public static BaseStatusSignal[] getFaultStatusSignals(TalonFXS talonFXS) {
			return new BaseStatusSignal[] {
				talonFXS.getFault_BootDuringEnable(),
				talonFXS.getFault_BridgeBrownout(),
				talonFXS.getFault_BridgeShort(),
				talonFXS.getFault_DeviceTemp(),
				talonFXS.getFault_DriveDisabledHallSensor(),
				talonFXS.getFault_ForwardHardLimit(),
				talonFXS.getFault_ForwardSoftLimit(),
				talonFXS.getFault_FusedSensorOutOfSync(),
				talonFXS.getFault_HallSensorMissing(),
				talonFXS.getFault_Hardware(),
				talonFXS.getFault_MissingDifferentialFX(),
				talonFXS.getFault_MissingHardLimitRemote(),
				talonFXS.getFault_MissingSoftLimitRemote(),
				talonFXS.getFault_MotorTempSensorMissing(),
				talonFXS.getFault_MotorTempSensorTooHot(),
				talonFXS.getFault_ProcTemp(),
				talonFXS.getFault_RemoteSensorDataInvalid(),
				talonFXS.getFault_RemoteSensorPosOverflow(),
				talonFXS.getFault_RemoteSensorReset(),
				talonFXS.getFault_ReverseHardLimit(),
				talonFXS.getFault_ReverseSoftLimit(),
				talonFXS.getFault_StaticBrakeDisabled(),
				talonFXS.getFault_StatorCurrLimit(),
				talonFXS.getFault_SupplyCurrLimit(),
				talonFXS.getFault_OverSupplyV(),
				talonFXS.getFault_Undervoltage(),
				talonFXS.getFault_UnstableSupplyV(),
				talonFXS.getFault_UsingFusedCANcoderWhileUnlicensed(),
				talonFXS.getFault_UnlicensedFeatureInUse(),
			};
		}
		public static BaseStatusSignal[] getStickyFaultStatusSignals(TalonFXS talonFXS) {
			return new BaseStatusSignal[] {
				talonFXS.getStickyFault_BootDuringEnable(),
				talonFXS.getStickyFault_BridgeBrownout(),
				talonFXS.getStickyFault_BridgeShort(),
				talonFXS.getStickyFault_DeviceTemp(),
				talonFXS.getStickyFault_DriveDisabledHallSensor(),
				talonFXS.getStickyFault_ForwardHardLimit(),
				talonFXS.getStickyFault_ForwardSoftLimit(),
				talonFXS.getStickyFault_FusedSensorOutOfSync(),
				talonFXS.getStickyFault_HallSensorMissing(),
				talonFXS.getStickyFault_Hardware(),
				talonFXS.getStickyFault_MissingDifferentialFX(),
				talonFXS.getStickyFault_MissingHardLimitRemote(),
				talonFXS.getStickyFault_MissingSoftLimitRemote(),
				talonFXS.getStickyFault_MotorTempSensorMissing(),
				talonFXS.getStickyFault_MotorTempSensorTooHot(),
				talonFXS.getStickyFault_ProcTemp(),
				talonFXS.getStickyFault_RemoteSensorDataInvalid(),
				talonFXS.getStickyFault_RemoteSensorPosOverflow(),
				talonFXS.getStickyFault_RemoteSensorReset(),
				talonFXS.getStickyFault_ReverseHardLimit(),
				talonFXS.getStickyFault_ReverseSoftLimit(),
				talonFXS.getStickyFault_StaticBrakeDisabled(),
				talonFXS.getStickyFault_StatorCurrLimit(),
				talonFXS.getStickyFault_SupplyCurrLimit(),
				talonFXS.getStickyFault_OverSupplyV(),
				talonFXS.getStickyFault_Undervoltage(),
				talonFXS.getStickyFault_UnstableSupplyV(),
				talonFXS.getStickyFault_UsingFusedCANcoderWhileUnlicensed(),
				talonFXS.getStickyFault_UnlicensedFeatureInUse(),
			};
		}
		public boolean getFaultFrom(TalonFXS talonFXS) {
			return switch (this) {
				case BootDuringEnable -> talonFXS.getFault_BootDuringEnable().getValue();
				case BridgeShort -> talonFXS.getFault_BridgeShort().getValue();
				case BridgeBrownout -> talonFXS.getFault_BridgeBrownout().getValue();
				case DeviceTemperature -> talonFXS.getFault_DeviceTemp().getValue();
				case DriveDisabledHallSensor -> talonFXS.getFault_DriveDisabledHallSensor().getValue();
				case ForwardHardLimit -> talonFXS.getFault_ForwardHardLimit().getValue();
				case ForwardSoftLimit -> talonFXS.getFault_ForwardSoftLimit().getValue();
				case FusedCancoderOutOfSync -> talonFXS.getFault_FusedSensorOutOfSync().getValue();
				case HallSensorMissing -> talonFXS.getFault_HallSensorMissing().getValue();
				case Hardware -> talonFXS.getFault_Hardware().getValue();
				case MissingDifferentialTalonFX -> talonFXS.getFault_MissingDifferentialFX().getValue();
				case MissingRemoteLimitSwitch -> talonFXS.getFault_MissingHardLimitRemote().getValue();
				case MissingRemoteSoftLimit -> talonFXS.getFault_MissingSoftLimitRemote().getValue();
				case MotorTempSensorMissing -> talonFXS.getFault_MotorTempSensorMissing().getValue();
				case MotorTempSensorTooHot -> talonFXS.getFault_MotorTempSensorTooHot().getValue();
				case ProcessorTemperature -> talonFXS.getFault_ProcTemp().getValue();
				case RemoteSensorDataInvalid -> talonFXS.getFault_RemoteSensorDataInvalid().getValue();
				case RemoteSensorPositionOverflow -> talonFXS.getFault_RemoteSensorPosOverflow().getValue();
				case RemoteSensorReset -> talonFXS.getFault_RemoteSensorReset().getValue();
				case ReverseHardLimit -> talonFXS.getFault_ReverseHardLimit().getValue();
				case ReverseSoftLimit -> talonFXS.getFault_ReverseSoftLimit().getValue();
				case StaticBrakeDisabled -> talonFXS.getFault_StaticBrakeDisabled().getValue();
				case StatorCurrentLimit -> talonFXS.getFault_StatorCurrLimit().getValue();
				case SupplyCurrentLimit -> talonFXS.getFault_SupplyCurrLimit().getValue();
				case SupplyOvervoltage -> talonFXS.getFault_OverSupplyV().getValue();
				case Undervoltage -> talonFXS.getFault_Undervoltage().getValue();
				case UnstableSupplyVoltage -> talonFXS.getFault_UnstableSupplyV().getValue();
				case UsingFusedCancoderWhileUnlicensed -> talonFXS.getFault_UsingFusedCANcoderWhileUnlicensed().getValue();
				case UsingUnlicensedFeature -> talonFXS.getFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public boolean getStickyFaultFrom(TalonFXS talonFXS) {
			return switch (this) {
				case BootDuringEnable -> talonFXS.getStickyFault_BootDuringEnable().getValue();
				case BridgeBrownout -> talonFXS.getStickyFault_BridgeBrownout().getValue();
				case BridgeShort -> talonFXS.getStickyFault_BridgeShort().getValue();
				case DeviceTemperature -> talonFXS.getStickyFault_DeviceTemp().getValue();
				case DriveDisabledHallSensor -> talonFXS.getStickyFault_DriveDisabledHallSensor().getValue();
				case ForwardHardLimit -> talonFXS.getStickyFault_ForwardHardLimit().getValue();
				case ForwardSoftLimit -> talonFXS.getStickyFault_ForwardSoftLimit().getValue();
				case FusedCancoderOutOfSync -> talonFXS.getStickyFault_FusedSensorOutOfSync().getValue();
				case HallSensorMissing -> talonFXS.getStickyFault_HallSensorMissing().getValue();
				case Hardware -> talonFXS.getStickyFault_Hardware().getValue();
				case MissingDifferentialTalonFX -> talonFXS.getStickyFault_MissingDifferentialFX().getValue();
				case MissingRemoteLimitSwitch -> talonFXS.getStickyFault_MissingHardLimitRemote().getValue();
				case MissingRemoteSoftLimit -> talonFXS.getStickyFault_MissingSoftLimitRemote().getValue();
				case MotorTempSensorMissing -> talonFXS.getStickyFault_MotorTempSensorMissing().getValue();
				case MotorTempSensorTooHot -> talonFXS.getStickyFault_MotorTempSensorTooHot().getValue();
				case ProcessorTemperature -> talonFXS.getStickyFault_ProcTemp().getValue();
				case RemoteSensorDataInvalid -> talonFXS.getStickyFault_RemoteSensorDataInvalid().getValue();
				case RemoteSensorPositionOverflow -> talonFXS.getStickyFault_RemoteSensorPosOverflow().getValue();
				case RemoteSensorReset -> talonFXS.getStickyFault_RemoteSensorReset().getValue();
				case ReverseHardLimit -> talonFXS.getStickyFault_ReverseHardLimit().getValue();
				case ReverseSoftLimit -> talonFXS.getStickyFault_ReverseSoftLimit().getValue();
				case StaticBrakeDisabled -> talonFXS.getStickyFault_StaticBrakeDisabled().getValue();
				case StatorCurrentLimit -> talonFXS.getStickyFault_StatorCurrLimit().getValue();
				case SupplyCurrentLimit -> talonFXS.getStickyFault_SupplyCurrLimit().getValue();
				case SupplyOvervoltage -> talonFXS.getStickyFault_OverSupplyV().getValue();
				case Undervoltage -> talonFXS.getStickyFault_Undervoltage().getValue();
				case UnstableSupplyVoltage -> talonFXS.getStickyFault_UnstableSupplyV().getValue();
				case UsingFusedCancoderWhileUnlicensed -> talonFXS.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue();
				case UsingUnlicensedFeature -> talonFXS.getStickyFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public StatusCode clearStickyFaultOn(TalonFXS talonFXS) {
			return switch (this) {
				case BootDuringEnable -> talonFXS.clearStickyFault_BootDuringEnable();
				case BridgeBrownout -> talonFXS.clearStickyFault_BridgeBrownout();
				case BridgeShort -> talonFXS.clearStickyFault_BridgeShort();
				case DeviceTemperature -> talonFXS.clearStickyFault_DeviceTemp();
				case DriveDisabledHallSensor -> talonFXS.clearStickyFault_DriveDisabledHallSensor();
				case ForwardHardLimit -> talonFXS.clearStickyFault_ForwardHardLimit();
				case ForwardSoftLimit -> talonFXS.clearStickyFault_ForwardSoftLimit();
				case FusedCancoderOutOfSync -> talonFXS.clearStickyFault_FusedSensorOutOfSync();
				case HallSensorMissing -> talonFXS.clearStickyFault_HallSensorMissing();
				case Hardware -> talonFXS.clearStickyFault_Hardware();
				case MissingDifferentialTalonFX -> talonFXS.clearStickyFault_MissingDifferentialFX();
				case MissingRemoteLimitSwitch -> talonFXS.clearStickyFault_MissingHardLimitRemote();
				case MissingRemoteSoftLimit -> talonFXS.clearStickyFault_MissingSoftLimitRemote();
				case MotorTempSensorMissing -> talonFXS.clearStickyFault_MotorTempSensorMissing();
				case MotorTempSensorTooHot -> talonFXS.clearStickyFault_MotorTempSensorTooHot();
				case ProcessorTemperature -> talonFXS.clearStickyFault_ProcTemp();
				case RemoteSensorDataInvalid -> talonFXS.clearStickyFault_RemoteSensorDataInvalid();
				case RemoteSensorPositionOverflow -> talonFXS.clearStickyFault_RemoteSensorPosOverflow();
				case RemoteSensorReset -> talonFXS.clearStickyFault_RemoteSensorReset();
				case ReverseHardLimit -> talonFXS.clearStickyFault_ReverseHardLimit();
				case ReverseSoftLimit -> talonFXS.clearStickyFault_ReverseSoftLimit();
				case StaticBrakeDisabled -> talonFXS.clearStickyFault_StaticBrakeDisabled();
				case StatorCurrentLimit -> talonFXS.clearStickyFault_StatorCurrLimit();
				case SupplyCurrentLimit -> talonFXS.clearStickyFault_SupplyCurrLimit();
				case SupplyOvervoltage -> talonFXS.clearStickyFault_OverSupplyV();
				case Undervoltage -> talonFXS.clearStickyFault_Undervoltage();
				case UnstableSupplyVoltage -> talonFXS.clearStickyFault_UnstableSupplyV();
				case UsingFusedCancoderWhileUnlicensed -> talonFXS.clearStickyFault_UsingFusedCANcoderWhileUnlicensed();
				case UsingUnlicensedFeature -> talonFXS.clearStickyFault_UnlicensedFeatureInUse();
				default -> StatusCode.OK;
			};
		}
		public static final FaultType[] possibleCancoderFaults = new FaultType[] {
			BadMagnet,
			BootDuringEnable,
			Hardware,
			Undervoltage,
			UsingUnlicensedFeature,
		};
		public static BaseStatusSignal[] getFaultStatusSignals(CANcoder cancoder) {
			return new BaseStatusSignal[] {
				cancoder.getFault_BadMagnet(),
				cancoder.getFault_BootDuringEnable(),
				cancoder.getFault_Hardware(),
				cancoder.getFault_Undervoltage(),
				cancoder.getFault_UnlicensedFeatureInUse(),
			};
		}
		public static BaseStatusSignal[] getStickyFaultStatusSignals(CANcoder cancoder) {
			return new BaseStatusSignal[] {
				cancoder.getStickyFault_BadMagnet(),
				cancoder.getStickyFault_BootDuringEnable(),
				cancoder.getStickyFault_Hardware(),
				cancoder.getStickyFault_Undervoltage(),
				cancoder.getStickyFault_UnlicensedFeatureInUse(),
			};
		}
		public boolean getFaultFrom(CANcoder cancoder) {
			return switch (this) {
				case BadMagnet -> cancoder.getFault_BadMagnet().getValue();
				case BootDuringEnable -> cancoder.getFault_BootDuringEnable().getValue();
				case Hardware -> cancoder.getFault_Hardware().getValue();
				case Undervoltage -> cancoder.getFault_Undervoltage().getValue();
				case UsingUnlicensedFeature -> cancoder.getFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public boolean getStickyFaultFrom(CANcoder cancoder) {
			return switch (this) {
				case BadMagnet -> cancoder.getStickyFault_BadMagnet().getValue();
				case BootDuringEnable -> cancoder.getStickyFault_BootDuringEnable().getValue();
				case Hardware -> cancoder.getStickyFault_Hardware().getValue();
				case Undervoltage -> cancoder.getStickyFault_Undervoltage().getValue();
				case UsingUnlicensedFeature -> cancoder.getStickyFault_UnlicensedFeatureInUse().getValue();
				default -> false;
			};
		}
		public StatusCode clearStickyFaultOn(CANcoder cancoder) {
			return switch (this) {
				case BadMagnet -> cancoder.clearStickyFault_BadMagnet();
				case BootDuringEnable -> cancoder.clearStickyFault_BootDuringEnable();
				case Hardware -> cancoder.clearStickyFault_Hardware();
				case Undervoltage -> cancoder.clearStickyFault_Undervoltage();
				case UsingUnlicensedFeature -> cancoder.clearStickyFault_UnlicensedFeatureInUse();
				default -> StatusCode.OK;
			};
		}
		public static final FaultType[] possibleSparkMaxFaults = new FaultType[] {
			CAN,
			EscEepromFault,
			EscEepromWarning,
			ExtEeprom,
			GateDriver,
			Hardware,
			HasReset,
			MotorType,
			Overcurrent,
			SensorFault,
			SensorWarning,
			Stall,
			Temperature,
			Undervoltage,
			UnknownFault,
			UnknownWarning,
		};
		public boolean getFaultFrom(SparkMax sparkMax) {
			return switch (this) {
				case CAN -> sparkMax.getFaults().can;
				case EscEepromFault -> sparkMax.getFaults().escEeprom;
				case EscEepromWarning -> sparkMax.getWarnings().escEeprom;
				case ExtEeprom -> sparkMax.getWarnings().extEeprom;
				case GateDriver -> sparkMax.getFaults().gateDriver;
				case Hardware -> sparkMax.getFaults().firmware;
				case HasReset -> sparkMax.getWarnings().hasReset;
				case MotorType -> sparkMax.getFaults().motorType;
				case Overcurrent -> sparkMax.getWarnings().overcurrent;
				case SensorFault -> sparkMax.getFaults().sensor;
				case SensorWarning -> sparkMax.getWarnings().sensor;
				case Stall -> sparkMax.getWarnings().stall;
				case Temperature -> sparkMax.getFaults().temperature;
				case Undervoltage -> sparkMax.getWarnings().brownout;
				case UnknownFault -> sparkMax.getFaults().other;
				case UnknownWarning -> sparkMax.getWarnings().other;
				default -> false;
			};
		}
		public boolean getStickyFaultFrom(SparkMax sparkMax) {
			return switch (this) {
				case CAN -> sparkMax.getStickyFaults().can;
				case EscEepromFault -> sparkMax.getStickyFaults().escEeprom;
				case EscEepromWarning -> sparkMax.getStickyWarnings().escEeprom;
				case ExtEeprom -> sparkMax.getStickyWarnings().extEeprom;
				case GateDriver -> sparkMax.getStickyFaults().gateDriver;
				case Hardware -> sparkMax.getStickyFaults().firmware;
				case HasReset -> sparkMax.getStickyWarnings().hasReset;
				case MotorType -> sparkMax.getStickyFaults().motorType;
				case Overcurrent -> sparkMax.getStickyWarnings().overcurrent;
				case SensorFault -> sparkMax.getStickyFaults().sensor;
				case SensorWarning -> sparkMax.getStickyWarnings().sensor;
				case Stall -> sparkMax.getStickyWarnings().stall;
				case Temperature -> sparkMax.getStickyFaults().temperature;
				case Undervoltage -> sparkMax.getStickyWarnings().brownout;
				case UnknownFault -> sparkMax.getStickyFaults().other;
				case UnknownWarning -> sparkMax.getStickyWarnings().other;
				default -> false;
			};
		}
	}
}

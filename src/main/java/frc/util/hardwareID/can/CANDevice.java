package frc.util.hardwareID.can;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.util.hardwareID.PnuematicsControl;

public class CANDevice {
	public final int id;
	public final CANBus bus;

	private CANDevice(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}

	public static CANDevice id(int id, CANBus bus) {
		return new CANDevice(id, bus);
	}

	// CTRE
	// | Phoenix 6
	public TalonFX talonFX() {
		return new TalonFX(id, bus.name);
	}
	public TalonFXS talonFXS() {
		return new TalonFXS(id, bus.name);
	}
	public CANcoder cancoder() {
		return new CANcoder(id, bus.name);
	}
	public Pigeon2 pigeon2() {
		return new Pigeon2(id, bus.name);
	}
	public CANdi candi() {
		return new CANdi(id, bus.name);
	}
	// | Phoenix 5
	public TalonSRX talonSRX() {
		return new TalonSRX(id);
	}
	public VictorSPX victorSPX() {
		return new VictorSPX(id);
	}
	public PnuematicsControl pneumaticsControlModule() {
		return pnuematicsControl(PneumaticsModuleType.CTREPCM);
	}
	// REV
	public SparkMax sparkMax(MotorType motorType) {
		return new SparkMax(id, motorType);
	}
	public PnuematicsControl pneumaticHub() {
		return pnuematicsControl(PneumaticsModuleType.REVPH);
	}
	// Grapple
	public LaserCan laserCan() {
		return new LaserCan(id);
	}

	public PnuematicsControl pnuematicsControl(PneumaticsModuleType type) {
		return new PnuematicsControl(this, type);
	}
}

package frc.robot;

import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Watts;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class BatteryLogger {
	private static BatteryLogger instance = null;
	public static BatteryLogger getInstance() {if (instance == null) {instance = new BatteryLogger();} return instance;}

	private double prevTimestamp = -1.0;
	private double totalEnergyJoules = 0.0;
	private double totalPowerWatts = 0.0;

	private final HashMap<String, Double> mechanismEnergiesJoules = new HashMap<>();

	public void logMechanism(String name, double totalSupplyCurrent) {
		if (this.prevTimestamp == -1.0) {
			return;
		}
		final var timestamp = Timer.getTimestamp();
		final var dtSeconds = timestamp - this.prevTimestamp;
		final var powerWatts = totalSupplyCurrent * RobotController.getBatteryVoltage();
		final var prevEnergyJoules = this.mechanismEnergiesJoules.getOrDefault(name, 0.0);
		final var deltaEnergyJoules = powerWatts * dtSeconds;
		final var energyJoules = deltaEnergyJoules + prevEnergyJoules;
		this.mechanismEnergiesJoules.put(name, energyJoules);
		this.totalEnergyJoules += deltaEnergyJoules;
		this.totalPowerWatts += powerWatts;

		Logger.recordOutput("BatteryLogger/" + name + "/Power Watts", powerWatts, Watts);
		Logger.recordOutput("BatteryLogger/" + name + "/Energy Joules", energyJoules, Joules);
	}

	public void periodic() {
		final var timestamp = Timer.getTimestamp();
		if (this.prevTimestamp == -1.0) {
			this.prevTimestamp = timestamp;
			return;
		}

		Logger.recordOutput("BatteryLogger/Total Power Watts", this.totalPowerWatts, Watts);
		Logger.recordOutput("BatteryLogger/Total Energy Joules", this.totalEnergyJoules, Joules);

		this.totalPowerWatts = 0.0;
		this.prevTimestamp = timestamp;
	}
}

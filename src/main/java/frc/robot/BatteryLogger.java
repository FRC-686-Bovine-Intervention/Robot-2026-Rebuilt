package frc.robot;

import static edu.wpi.first.units.Units.Amps;
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
	private double totalCurrentAmps = 0.0;

	private final HashMap<String, Double> mechanismEnergiesJoules = new HashMap<>();
	private final HashMap<String, Double> mechanismPowerWatts = new HashMap<>();

	public void logMechanism(String name, double totalSupplyCurrent) {
		if (this.prevTimestamp == -1.0) {
			return;
		}
		final var timestamp = Timer.getTimestamp();
		final var dtSeconds = timestamp - this.prevTimestamp;
		final var powerWatts = totalSupplyCurrent * RobotController.getBatteryVoltage();
		final var prevEnergyJoules = this.mechanismEnergiesJoules.getOrDefault(name, 0.0);
		final var prevPowerWatts = this.mechanismPowerWatts.getOrDefault(name, 0.0);
		final var midpointPowerWatts = (prevPowerWatts + powerWatts) / 2.0;
		final var deltaEnergyJoules = midpointPowerWatts * dtSeconds;
		final var energyJoules = deltaEnergyJoules + prevEnergyJoules;
		this.mechanismEnergiesJoules.put(name, energyJoules);
		this.mechanismPowerWatts.put(name, powerWatts);
		this.totalEnergyJoules += deltaEnergyJoules;
		this.totalPowerWatts += powerWatts;
		this.totalCurrentAmps += totalSupplyCurrent;

		Logger.recordOutput("BatteryLogger/" + name + "/Power Watts", powerWatts, Watts);
		Logger.recordOutput("BatteryLogger/" + name + "/Energy Joules", energyJoules, Joules);
		Logger.recordOutput("BatteryLogger/" + name + "/Current Amps", totalSupplyCurrent, Amps);
	}

	public void periodic() {
		final var timestamp = Timer.getTimestamp();
		if (this.prevTimestamp == -1.0) {
			this.prevTimestamp = timestamp;
			return;
		}

		Logger.recordOutput("BatteryLogger/Total Power Watts", this.totalPowerWatts, Watts);
		Logger.recordOutput("BatteryLogger/Total Energy Joules", this.totalEnergyJoules, Joules);
		Logger.recordOutput("BatteryLogger/Total Current Amps", this.totalCurrentAmps, Amps);

		this.totalPowerWatts = 0.0;
		this.totalCurrentAmps = 0.0;
		this.prevTimestamp = timestamp;
	}
}

package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class WheelRadiusCalibration extends Command {
	private final Drive drive;
	private final Timer totalTimer = new Timer();
	private final Voltage maxVoltage;
	private final Velocity<VoltageUnit> voltageRampRate;
	private final double[] initialPositionRads = new double[DriveConstants.moduleConstants.length];
	private double prevYawRads = 0.0;
	private double totalYawRads = 0.0;

	private static final VelocityUnit<VoltageUnit> VOLTAGE_RAMP_UNIT = Volts.per(Second);
	public static final LoggedTunable<Velocity<VoltageUnit>> VOLTAGE_RAMP_RATE = LoggedTunable.from("Drive/Wheel Calibration/Voltage Ramp Rate", VOLTAGE_RAMP_UNIT::of, 2);
	public static final LoggedTunable<Voltage> MAX_VOLTAGE = LoggedTunable.from("Drive/Wheel Calibration/Max Voltage", Volts::of, 6);

	public WheelRadiusCalibration(Drive drive, Velocity<VoltageUnit> voltageRampRate, Voltage maxVoltage) {
		this.drive = drive;
		this.addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
		this.setName("Wheel Calibration");
		this.voltageRampRate = voltageRampRate;
		this.maxVoltage = maxVoltage;
	}

	@Override
	public void initialize() {
		this.totalTimer.restart();
		this.prevYawRads = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
		this.totalYawRads = 0.0;
		for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
			this.initialPositionRads[i] = this.drive.modules[i].getWheelAngularPositionRads();
		}
	}

	@Override
	public void execute() {
		var yawRads = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
		var yawDiff = yawRads - this.prevYawRads;
		var wrappedDiff = MathUtil.angleModulus(yawDiff);
		this.totalYawRads += wrappedDiff;

		this.prevYawRads = yawRads;

		var averageWheelTravelRads = 0.0;
		for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
			averageWheelTravelRads += this.drive.modules[i].getWheelAngularPositionRads() - this.initialPositionRads[i];
		}

		averageWheelTravelRads /= DriveConstants.moduleConstants.length;

		var expectedWheelTravelMeters = DriveConstants.driveBaseRadius.in(Meters) * this.totalYawRads;

		var averageWheelRadiusMeters = expectedWheelTravelMeters / averageWheelTravelRads;

		Logger.recordOutput("Drive/Wheel Calibration/Total Yaw", this.totalYawRads, Radians);
		Logger.recordOutput("Drive/Wheel Calibration/Average Wheel Travel", averageWheelTravelRads, Radians);
		Logger.recordOutput("Drive/Wheel Calibration/Expected Wheel Travel", expectedWheelTravelMeters, Meters);
		Logger.recordOutput("Drive/Wheel Calibration/Average Wheel Radius", averageWheelRadiusMeters, Meters);

		var volts = Math.min(this.voltageRampRate.in(VOLTAGE_RAMP_UNIT) * this.totalTimer.get(), this.maxVoltage.in(Volts));
		for (var module : this.drive.modules) {
			module.runVolts(volts, module.config.positiveRotVec);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.totalTimer.stop();
		this.drive.stop();
	}
}

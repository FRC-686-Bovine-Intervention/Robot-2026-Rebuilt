package frc.robot.subsystems.drive.modules;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
	private final DCMotor driveMotorModel = DCMotor.getFalcon500(1);
	private final DCMotor azimuthMotorModel = DCMotor.getNeo550(1);

	private final DCMotorSim driveSim = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(this.driveMotorModel, 0.0025, DriveConstants.driveMotorToWheelRatio.reductionUnsigned()),
		this.azimuthMotorModel
	);
	private final DCMotorSim azimuthSim = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(this.azimuthMotorModel, 0.0004, DriveConstants.azimuthMotorToCarriageRatio.reductionUnsigned()),
		this.azimuthMotorModel
	);

	public ModuleIOSim(ModuleConstants moduleConstants) {
		super(moduleConstants);
	}

	private final MutVoltage azimuthAppliedVolts = Volts.mutable(0);

	public void updateInputs(ModuleIOInputs inputs) {
		var driveSimState = this.driveMotor.getSimState();
		if (DriverStation.isDisabled()) {
			this.azimuthAppliedVolts.mut_setBaseUnitMagnitude(0);
		}
		this.driveSim.setInputVoltage(driveSimState.getMotorVoltage());
		this.azimuthSim.setInputVoltage(this.azimuthAppliedVolts.in(Volts));

		this.driveSim.update(RobotConstants.rioUpdatePeriodSecs);
		this.azimuthSim.update(RobotConstants.rioUpdatePeriodSecs);

		var wheelAngle = this.driveSim.getAngularPosition();
		var wheelVelocity = this.driveSim.getAngularVelocity();
		driveSimState.setRawRotorPosition(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelAngle));
		driveSimState.setRotorVelocity(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelVelocity));
		driveSimState.setSupplyVoltage(12 - driveSimState.getSupplyCurrent() * 0.002);

		super.updateInputs(inputs);

		var carriageAngle = this.azimuthSim.getAngularPosition();
		var carriageVelocity = this.azimuthSim.getAngularVelocity();
		inputs.azimuthEncoder.setPositionRads(DriveConstants.azimuthEncoderToCarriageRatio.inverse().applyUnsigned(carriageAngle).in(Radians));
		inputs.azimuthEncoder.setVelocityRadsPerSec(DriveConstants.azimuthEncoderToCarriageRatio.inverse().applyUnsigned(carriageVelocity).in(RadiansPerSecond));
		inputs.azimuthMotor.encoder.setPositionRads(DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageAngle).in(Radians));
		inputs.azimuthMotor.encoder.setVelocityRadsPerSec(DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageVelocity).in(RadiansPerSecond));
		inputs.azimuthMotor.motor.updateFrom(this.azimuthSim);

		inputs.odometryDriveRads = new double[] {inputs.driveMotor.encoder.getPositionRads()};
		inputs.odometryAzimuthRads = new double[] {inputs.azimuthEncoder.getPositionRads()};
	}

	@Override
	public void setAzimuthVolts(double volts) {
		this.azimuthAppliedVolts.mut_replace(MathUtil.clamp(volts, -12, 12), Volts);
	}
	@Override
	public void setAzimuthAngleRads(double angleRads) {
		this.setAzimuthVolts(
			this.azimuthPID.calculate(
				this.azimuthSim.getAngularPosition().in(Rotations),
				Units.radiansToRotations(angleRads)
			)
		);
	}
}

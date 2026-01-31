package frc.robot.subsystems.drive.modules;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
	private final DCMotor driveMotorModel = DCMotor.getFalcon500(1);
	private final DCMotor azimuthMotorModel = DCMotor.getNeo550(1);

	private final DCMotorSim driveSim = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(this.driveMotorModel, 0.005, DriveConstants.driveMotorToWheelRatio.reductionUnsigned()),
		this.azimuthMotorModel
	);
	private final DCMotorSim azimuthSim = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(this.azimuthMotorModel, 0.75, DriveConstants.azimuthMotorToCarriageRatio.reductionUnsigned()),
		this.azimuthMotorModel
	);

	private final SparkMaxSim azimuthSparkMaxSim = new SparkMaxSim(this.azimuthMotor, this.azimuthMotorModel);
	// private final SparkRelativeEncoderSim azimuthRelativeEncoderSim = this.azimuthSparkMaxSim.getRelativeEncoderSim();
	private final SparkAbsoluteEncoderSim azimuthAbsoluteEncoderSim = this.azimuthSparkMaxSim.getAbsoluteEncoderSim();

	public ModuleIOSim(ModuleConstants moduleConstants) {
		super(moduleConstants);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		var driveSimState = this.driveMotor.getSimState();
		this.driveSim.setInputVoltage(driveSimState.getMotorVoltage());
		this.azimuthSim.setInputVoltage(this.azimuthSparkMaxSim.getAppliedOutput() * 12.0);

		this.driveSim.update(RobotConstants.rioUpdatePeriodSecs);
		this.azimuthSim.update(RobotConstants.rioUpdatePeriodSecs);

		var wheelAngle = this.driveSim.getAngularPosition();
		var wheelVelocity = this.driveSim.getAngularVelocity();
		driveSimState.setRawRotorPosition(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelAngle));
		driveSimState.setRotorVelocity(DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(wheelVelocity));
		driveSimState.setSupplyVoltage(12.0 - driveSimState.getSupplyCurrent() * 0.002);

		var carriageAngle = this.azimuthSim.getAngularPosition();
		var carriageVelocity = this.azimuthSim.getAngularVelocity();
		this.azimuthAbsoluteEncoderSim.iterate(
			DriveConstants.azimuthEncoderToCarriageRatio.inverse().applyUnsigned(carriageVelocity).in(RPM),
			RobotConstants.rioUpdatePeriodSecs
		);
		// this.azimuthRelativeEncoderSim.iterate(
		// 	DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageVelocity).in(RPM),
		// 	RobotConstants.rioUpdatePeriodSecs
		// );
		this.azimuthSparkMaxSim.iterate(
			DriveConstants.azimuthMotorToCarriageRatio.inverse().applyUnsigned(carriageVelocity).in(RPM),
			12.0,
			RobotConstants.rioUpdatePeriodSecs
		);

		super.updateInputs(inputs);

		inputs.odometryDriveRads = new double[] {inputs.driveMotor.encoder.getPositionRads()};
		inputs.odometryAzimuthRads = new double[] {inputs.azimuthEncoder.getPositionRads()};
	}
}

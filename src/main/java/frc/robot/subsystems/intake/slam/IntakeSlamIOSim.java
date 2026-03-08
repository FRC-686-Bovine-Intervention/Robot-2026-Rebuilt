package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class IntakeSlamIOSim extends IntakeSlamIOTalonFX {
	private final SingleJointedArmSim slamSim = new SingleJointedArmSim(
		LinearSystemId.identifyPositionSystem(0.5, 0.1),
		DCMotor.getKrakenX60(1),
		IntakeSlamConstants.motorToMechanism.reductionUnsigned(),
		1,
		IntakeSlamConstants.minAngle.in(Radians),
		IntakeSlamConstants.maxAngle.in(Radians),
		true,
		IntakeSlamConstants.maxAngle.in(Radians)
	);

	@Override
	public void updateInputs(IntakeSlamIOInputs inputs) {
		var motorSimState = this.motor.getSimState();
		var encoderSimState = this.encoder.getSimState();

		this.slamSim.setInputVoltage(-motorSimState.getMotorVoltage());
		this.slamSim.update(RobotConstants.rioUpdatePeriodSecs);

		var mechAngle = Radians.of(this.slamSim.getAngleRads());
		var mechVelo = RadiansPerSecond.of(this.slamSim.getVelocityRadPerSec());

		encoderSimState.setRawPosition(IntakeSlamConstants.sensorToMechanism.inverse().applyUnsigned(mechAngle.minus(IntakeSlamConstants.encoderZeroOffset)));
		encoderSimState.setVelocity(IntakeSlamConstants.sensorToMechanism.inverse().applyUnsigned(mechVelo));

		motorSimState.setRawRotorPosition(IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(mechAngle));
		motorSimState.setRotorVelocity(IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(mechVelo));

		motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		super.updateInputs(inputs);
	}
}

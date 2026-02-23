package frc.robot.subsystems.climber.hook;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.RobotConstants;

public class HookIOSim extends HookIOTalonFX {
	private final ElevatorSim hookSim = new ElevatorSim(
		LinearSystemId.identifyPositionSystem(12, 12),
		DCMotor.getFalcon500Foc(1).withReduction(HookConstants.motorToMechanism.reductionUnsigned()),
		HookConstants.minLength.in(Meters),
		HookConstants.maxLength.in(Meters),
		false,
		HookConstants.minLength.in(Meters)
	);

	@Override
	public void updateInputs(HookIOInputs inputs) {
		var motorSimState = this.motor.getSimState();

		this.hookSim.setInputVoltage(motorSimState.getMotorVoltage());
		this.hookSim.update(RobotConstants.rioUpdatePeriodSecs);

		var mechLength = Meters.of(this.hookSim.getPositionMeters());
		var mechVelo = MetersPerSecond.of(this.hookSim.getVelocityMetersPerSecond());

		var spoolAngle = HookConstants.spool.distanceToAngle(mechLength);
		var spoolVelo = HookConstants.spool.linearVelocityToAngularVelocity(mechVelo);

		motorSimState.setRawRotorPosition(HookConstants.motorToMechanism.inverse().applyUnsigned(spoolAngle));
		motorSimState.setRotorVelocity(HookConstants.motorToMechanism.inverse().applyUnsigned(spoolVelo));

		motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		motorSimState.setReverseLimit(this.hookSim.hasHitLowerLimit());

		super.updateInputs(inputs);
	}
}

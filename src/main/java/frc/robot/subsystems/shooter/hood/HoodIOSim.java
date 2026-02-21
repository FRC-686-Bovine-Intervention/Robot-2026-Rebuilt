package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.hook.HookConstants;

public class HoodIOSim extends HoodIOTalonFXS {
	private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
		LinearSystemId.identifyPositionSystem(12, 12),
		DCMotor.getNeo550(1),
		HoodConstants.motorToMechanism.reductionUnsigned(),
		0.5,
		HoodConstants.minAngle.in(Radians),
		HoodConstants.maxAngle.in(Radians),
		false,
		HoodConstants.minAngle.in(Radians)
	);

	private final CANdi candi;

	public HoodIOSim(CANdi candi) {
		super(candi);
		this.candi = candi;
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		var motorSimState = this.motor.getSimState();
		var candiSimState = candi.getSimState();

		this.hoodSim.setInputVoltage(motorSimState.getMotorVoltage());
		this.hoodSim.update(RobotConstants.rioUpdatePeriodSecs);

		var mechAngle = Radians.of(this.hoodSim.getAngleRads());
		var mechVelo = RadiansPerSecond.of(this.hoodSim.getVelocityRadPerSec());

		motorSimState.setRawRotorPosition(HookConstants.motorToMechanism.inverse().applyUnsigned(mechAngle));
		motorSimState.setRotorVelocity(HookConstants.motorToMechanism.inverse().applyUnsigned(mechVelo));

		motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		candiSimState.setS1State(
			(this.hoodSim.hasHitLowerLimit()) ? (
				S1StateValue.Low
			) : (
				S1StateValue.Floating
			)
		);

		super.updateInputs(inputs);
	}
}

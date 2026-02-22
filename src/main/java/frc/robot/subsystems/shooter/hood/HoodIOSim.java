package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.subsystems.commonDevices.CommonCANdi;

public class HoodIOSim extends HoodIOTalonFXS {
	private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
		LinearSystemId.identifyPositionSystem(3, 3),
		DCMotor.getNeo550(1).withReduction(HoodConstants.motorToMechanism.reductionUnsigned()),
		1.0,
		0.5,
		HoodConstants.minAngle.in(Radians),
		HoodConstants.maxAngle.plus(Degrees.of(180)).in(Radians),
		false,
		HoodConstants.minAngle.in(Radians)
	);

	private final CANdi candi;

	public HoodIOSim(CommonCANdi candi) {
		super(candi);
		this.candi = candi.candi;
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		var motorSimState = this.motor.getSimState();
		var candiSimState = this.candi.getSimState();

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

		inputs.limitSwitch = this.hoodSim.hasHitLowerLimit();
	}
}

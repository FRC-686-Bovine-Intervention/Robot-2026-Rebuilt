package frc.robot.subsystems.intake.rollers;

import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;

public class IntakeRollersIOSparkMax implements IntakeRollersIO {
	private final SparkMax motor = HardwareDevices.intakeRightRollerMotorID.sparkMax(MotorType.kBrushless);

	public IntakeRollersIOSparkMax() {
		var config = new SparkMaxConfig();
		config
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.smartCurrentLimit(40)
		;
		this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(IntakeRollersIOInputs inputs) {
		inputs.motor.updateFrom(this.motor);
	}

	@Override
	public void setVolts(double volts) {
		this.motor.setVoltage(volts);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		this.motor.stopMotor();
	}
}

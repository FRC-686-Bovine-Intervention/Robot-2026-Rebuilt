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
	private final SparkMax leftMotor = HardwareDevices.intakeLeftRollerMotorID.sparkMax(MotorType.kBrushless);
	private final SparkMax rightMotor = HardwareDevices.intakeRightRollerMotorID.sparkMax(MotorType.kBrushless);

	public IntakeRollersIOSparkMax() {
		final var leftConfig = new SparkMaxConfig();
		leftConfig
			.idleMode(IdleMode.kBrake)
			.inverted(false)
			.smartCurrentLimit(70)
		;
		this.leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		final var rightConfig = new SparkMaxConfig();
		rightConfig
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.smartCurrentLimit(70)
		;
		this.rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(IntakeRollersIOInputs inputs) {
		inputs.leftMotor.updateFrom(this.leftMotor);
		inputs.rightMotor.updateFrom(this.rightMotor);
	}

	@Override
	public void setVolts(double volts) {
		this.leftMotor.setVoltage(volts);
		this.rightMotor.setVoltage(volts);
	}

	@Override
	public void stop(Optional<NeutralMode> neutralMode) {
		this.leftMotor.stopMotor();
		this.rightMotor.stopMotor();
	}
}

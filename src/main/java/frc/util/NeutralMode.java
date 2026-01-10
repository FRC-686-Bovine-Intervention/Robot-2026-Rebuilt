package frc.util;

import java.util.Optional;

import com.ctre.phoenix6.controls.ControlRequest;

public enum NeutralMode {
	Coast(
		com.ctre.phoenix6.signals.NeutralModeValue.Coast,
		new com.ctre.phoenix6.controls.CoastOut(),
		com.ctre.phoenix.motorcontrol.NeutralMode.Coast,
		com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast
	),
	Brake(
		com.ctre.phoenix6.signals.NeutralModeValue.Brake,
		new com.ctre.phoenix6.controls.StaticBrake(),
		com.ctre.phoenix.motorcontrol.NeutralMode.Brake,
		com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
	),
	;
	private final com.ctre.phoenix6.signals.NeutralModeValue phoenix6NeutralMode;
	private final com.ctre.phoenix6.controls.ControlRequest phoenix6ControlRequest;
	private final com.ctre.phoenix.motorcontrol.NeutralMode phoenix5NeutralMode;
	private final com.revrobotics.spark.config.SparkBaseConfig.IdleMode revLibIdleMode;

	NeutralMode(
		com.ctre.phoenix6.signals.NeutralModeValue phoenix6NeutralMode,
		com.ctre.phoenix6.controls.ControlRequest phoenix6ControlRequest,
		com.ctre.phoenix.motorcontrol.NeutralMode phoenix5NeutralMode,
		com.revrobotics.spark.config.SparkBaseConfig.IdleMode revLibIdleMode
	) {
		this.phoenix6NeutralMode = phoenix6NeutralMode;
		this.phoenix6ControlRequest = phoenix6ControlRequest;
		this.phoenix5NeutralMode = phoenix5NeutralMode;
		this.revLibIdleMode = revLibIdleMode;
	}

	public com.ctre.phoenix6.signals.NeutralModeValue getPhoenix6NeutralMode() {
		return this.phoenix6NeutralMode;
	}
	public com.ctre.phoenix.motorcontrol.NeutralMode getPhoenix5NeutralMode() {
		return this.phoenix5NeutralMode;
	}
	public com.revrobotics.spark.config.SparkBaseConfig.IdleMode getRevLibIdleMode() {
		return this.revLibIdleMode;
	}

	public com.ctre.phoenix6.controls.ControlRequest getPhoenix6ControlRequest() {
		return this.phoenix6ControlRequest;
	}

	public static final Optional<NeutralMode> DEFAULT = Optional.empty();
	public static final Optional<NeutralMode> COAST = Optional.of(NeutralMode.Coast);
	public static final Optional<NeutralMode> BRAKE = Optional.of(NeutralMode.Brake);

	public static ControlRequest selectControlRequest(Optional<NeutralMode> neutralMode, ControlRequest defaultRequest, ControlRequest coastRequest, ControlRequest brakeRequest) {
		if (neutralMode.isEmpty()) {return defaultRequest;}
		return switch (neutralMode.get()) {
			case Coast -> coastRequest;
			case Brake -> brakeRequest;
		};
	}
}

package frc.robot.subsystems.climber.hook;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Hook extends SubsystemBase {
	private final HookIO io;
	private final HookIOInputsAutoLogged inputs = new HookIOInputsAutoLogged();

	private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
		//REVIEW THIS (39)
		"climber/hook/Profile",
		InchesPerSecond,
		InchesPerSecond.per(Second),
		MetersPerSecond,
		MetersPerSecondPerSecond,
		new TrapezoidProfile.Constraints(
			120,
			240
		)
	);
	private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
		"Climber/Hook/FF",
		new FFConstants(
			0.8,
			0.4,
			2,
			0
		)
	);
	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Climber/Hook/PID",
		new PIDConstants(
			50,
			0,
			0
		)
	);

	//private static final LoggedTunable<Distance> autoRezeroMaxLength = LoggedTunable.from("Superstructure/Elevator/Auto Rezero/Max Length", Inches::of, 3);
	//private static final LoggedTunable<Current> autoRezeroTorqueCurrentThreshold = LoggedTunable.from("Superstructure/Elevator/Auto Rezero/Torque Current Threshold", Amps::of, -50);
	//private static final LoggedTunable<LinearVelocity> autoRezeroMaxVelo = LoggedTunable.from("Superstructure/Elevator/Auto Rezero/Max Velocity", InchesPerSecond::of, 0.5);
	//private static final LoggedTunable<Time> autoRezeroDebounceTime = LoggedTunable.from("Superstructure/Elevator/Auto Rezero/Debounce Time", Seconds::of, 1);
	//private final Timer autoRezeroDebounceTimer = new Timer();

	private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConsts.get());
	private final State measuredState = new State();
	private final State setpointState = new State();
	private final State goalState = new State();
	private boolean motionProfiling = false;
	private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0,0,0,0);

	private double lengthMeters = 0.0;
	private double velocityMetersPerSec = 0.0;

	//public final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
	//public final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
	//public final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);

	private final Alert motorDisconnectedAlert = new Alert("climber/hook/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert encoderDisconnectedAlert = new Alert("climber/hook/Alerts", "Encoder Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Hook Motor Disconnected!", AlertType.kError);
	private final Alert encoderDisconnectedGlobalAlert = new Alert("Hook Encoder Disconnected!", AlertType.kError);

	public Hook(HookIO io) {
		System.out.println("[Init Elevator] Instantiating Elevator with " + io.getClass().getSimpleName());
		this.io = io;

		ffConsts.get().update(this.feedforward);
		this.io.configPID(pidConsts.get());

	}


	//REVIEW THIS (87-92)

	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/subsystems/climber/hook/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/subsystems/climber/hook/Update Inputs");
		Logger.processInputs("Inputs/Superstructure/Elevator", this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/subsystems/climber/hook/Process Inputs");

		var stageDistMeters = HookConstants.stage1LinearRelation.radiansToMeters(HookConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads()));

		this.lengthMeters = stageDistMeters * HookConstants.movingStageCount;
		this.velocityMetersPerSec = HookConstants.stage1LinearRelation.radiansToMeters(HookConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec())) * HookConstants.movingStageCount;

		//this.stage2Mech.setMeters(stageDistMeters);
		//this.stage3Mech.setMeters(stageDistMeters);
		//this.stage4Mech.setMeters(stageDistMeters);

		if (profileConsts.hasChanged(hashCode())) {
			this.motionProfile = new TrapezoidProfile(profileConsts.get());
		}
		if (ffConsts.hasChanged(hashCode())) {
			ffConsts.get().update(this.feedforward);
		}
		if (pidConsts.hasChanged(hashCode())) {
			this.io.configPID(pidConsts.get());
		}

		//if (
			//Math.abs(this.getLengthMeters()) < autoRezeroMaxLength.get().in(Meters)
			//&& Math.abs(this.getVelocityMetersPerSec()) < autoRezeroMaxVelo.get().in(MetersPerSecond)
			//&& this.inputs.motor.motor.getTorqueCurrentAmps() < autoRezeroTorqueCurrentThreshold.get().in(Amps)
		//) {
			//this.autoRezeroDebounceTimer.start();
		//} else {
			//this.autoRezeroDebounceTimer.stop();
			//this.autoRezeroDebounceTimer.reset();
		//}
		//if (this.autoRezeroDebounceTimer.hasElapsed(autoRezeroDebounceTime.get().in(Seconds))) {
			//this.io.configMagnetOffset(this.inputs.encoderMagnetOffsetRads - this.inputs.encoder.getPositionRads());
			//this.autoRezeroDebounceTimer.reset();
		//}

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.encoderDisconnectedAlert.set(!this.inputs.encoderConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);
		this.encoderDisconnectedGlobalAlert.set(!this.inputs.encoderConnected);


		//REVIEW THIA (137-138)
		LoggedTracer.logEpoch("CommandScheduler Periodic/subsystems/climber/Elevator/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/subsystems/climber/hook");
	}

	public double getLengthMeters() {
		return this.lengthMeters;
	}
	public double getVelocityMetersPerSec() {
		return this.velocityMetersPerSec;
	}
	public double getAppliedVolts() {
		return this.inputs.motor.motor.getAppliedVolts();
	}

	public void setVolts(double volts) {
		this.motionProfiling = false;
		this.io.setVolts(volts);
	}
	public void stop(Optional<NeutralMode> neutralMode) {
		this.motionProfiling = false;
		this.io.stop(neutralMode);
	}

	public void setLengthGoalMeters(double lengthMeters) {
		this.goalState.position = lengthMeters;
		this.goalState.velocity = 0.0;
		if (!this.motionProfiling) {
			this.setpointState.position = this.measuredState.position;
			this.setpointState.velocity = this.measuredState.velocity;
			this.motionProfiling = true;
		}

		var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
		var ffout = this.feedforward.calculateWithVelocities(this.setpointState.velocity, newSetpointState.velocity);

		this.setpointState.position = newSetpointState.position;
		this.setpointState.velocity = newSetpointState.velocity;
		this.io.setPosition(
			HookConstants.stage1LinearRelation.metersToRadians(this.setpointState.position / HookConstants.movingStageCount),
			HookConstants.stage1LinearRelation.metersToRadians(this.setpointState.velocity / HookConstants.movingStageCount),
			ffout
		);

		//REVIEW THIS (181-185)
		Logger.recordOutput("climber/hook/FF/FF Out", ffout);
		Logger.recordOutput("climber/hook/Length/Setpoint", this.setpointState.position);
		Logger.recordOutput("climber/hook/Velocity/Setpoint", this.setpointState.velocity);
		Logger.recordOutput("climber/hook/Length/Goal", goalState.position);
		Logger.recordOutput("climber/hook/Velocity/Goal", goalState.velocity);
	}
}

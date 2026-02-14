package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.EdgeDetector;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.ArmMech;

public class Hood extends SubsystemBase {
	private final HoodIO io;
	private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	private static final LoggedTunable<Angle> idleAngle = LoggedTunable.from("Shooter/Hood/Idle Angle", Degrees::of, HoodConstants.minAngle.in(Degrees));
    private static final LoggedTunable<Angle> rezeroThreshold = LoggedTunable.from("Shooter/Hood/Rezero Threshold", Degrees::of, 2.0);
    
    private static final LoggedTunable<ExponentialProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "Shooter/Hood/Profile",
        DegreesPerSecond,
        DegreesPerSecondPerSecond,
        RadiansPerSecond,
        ExponentialProfile.Constraints.fromCharacteristics(0, 0, 0)
    );

    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "Shooter/Hood/FF",
        new FFConstants(
            0.0,
            0.0,
            2.4,
            0.0
        )
    );

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Shooter/Hood/PID",
        new PIDConstants(
            1.5,
            0.0,
            0.0
        )
    );

    private double angleRads = 0.0;
    private double velocityRadsPerSec = 0.0;
    private double motorOffsetRads = 0.0;
    private boolean calibrated = false;

    private final EdgeDetector limitSwitchEdgeDetector = new EdgeDetector(false);

    public final ArmMech mech = new ArmMech(HoodConstants.hoodBase);

    private final Alert notCalibratedAlert = new Alert("Shooter/Hood/Alerts", "Not Calibrated", AlertType.kError);
    private final Alert notCalibratedGlobalAlert = new Alert("Hood Not Calibrated!", AlertType.kError);

    private final Alert motorDisconnectedAlert = new Alert("Shooter/Hood/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Hood Motor Disconnected!", AlertType.kError);

    public Hood(HoodIO io) {
        super("Shooter/Hood");

        System.out.println("[Init Hood] Instantiating Hood with " + io.getClass().getSimpleName());
        this.io = io;
		
        this.io.configPID(pidConsts.get());
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Update Inputs");
        Logger.processInputs("Inputs/Shooter/Hood", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Process Inputs");

        this.limitSwitchEdgeDetector.update(this.inputs.limitSwitch);
        if (this.limitSwitchEdgeDetector.risingEdge()) {
            this.resetInternalAngleRads(HoodConstants.minAngle.in(Radians));
            if (!this.calibrated || Math.abs(this.motorOffsetRads) >= rezeroThreshold.get().in(Radians)) {
                this.io.resetMotorPositionRads(HoodConstants.motorToMechanism.inverse().applyUnsigned(this.angleRads));
                this.motorOffsetRads = 0.0;
            }
            this.calibrated = true;
        }

        this.angleRads = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads()) + this.motorOffsetRads;
        this.velocityRadsPerSec = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

        this.measuredState.position = this.getAngleRads();
        this.measuredState.velocity = this.getVelocityRadsPerSec();

        this.mech.setRads(this.getAngleRads());

        Logger.recordOutput("Shooter/Hood/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Shooter/Hood/Velocity/Measured", this.getVelocityRadsPerSec());

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = new ExponentialProfile(profileConsts.get());
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.get().update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.get());
        }

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

        this.notCalibratedAlert.set(!this.calibrated);
        this.notCalibratedGlobalAlert.set(!this.calibrated);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood");
    }

    private void resetInternalAngleRads(double angleRads) {
        this.angleRads = angleRads;
        this.motorOffsetRads = this.angleRads - HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
    }

    public double getAngleRads() {
        return this.angleRads;
    }

    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }

    private void stop(Optional<NeutralMode> neutralMode) {
        this.motionProfiling = false;
        this.io.stop(neutralMode);
    }

    private void setAngleGoalRads(double angleRads) {
        this.goalState.position = angleRads;
        this.goalState.velocity = 0.0;
        if (!this.motionProfiling) {
            this.setpointState.position = this.measuredState.position;
            this.setpointState.velocity = this.measuredState.velocity;
            this.motionProfiling = true;
        }
        var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.position, this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState.position = newSetpointState.position;
        this.setpointState.velocity = newSetpointState.velocity;
        this.io.setPositionRads(
            HoodConstants.motorToMechanism.inverse().applyUnsigned(this.setpointState.position - this.motorOffsetRads),
            HoodConstants.motorToMechanism.inverse().applyUnsigned(this.setpointState.velocity),
            ffout
        );
        Logger.recordOutput("Shooter/Hood/FF/FF Out", ffout);
        Logger.recordOutput("Shooter/Hood/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Shooter/Hood/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Shooter/Hood/Angle/Goal", this.goalState.position);
        Logger.recordOutput("Shooter/Hood/Velocity/Goal", this.goalState.velocity);
    }

    public Command coast() {
        final var hood = this;
        return new Command() {
            {
                this.setName("Coast");
                this.addRequirements(hood);
            }

            @Override
            public void initialize() {
                hood.stop(NeutralMode.COAST);
            }

            @Override
            public void end(boolean interrupted) {
                hood.stop(NeutralMode.DEFAULT);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command genAngleCommand(String name, DoubleSupplier angleRads) {
        final var hood = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(hood);
            }

            @Override
            public void execute() {
                hood.setAngleGoalRads(angleRads.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                hood.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command idle() {
        return this.genAngleCommand(
            "Idle", 
            () -> idleAngle.get().in(Radians)
        );
    }
}

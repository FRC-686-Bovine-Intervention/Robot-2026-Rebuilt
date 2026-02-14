package frc.robot.subsystems.shooter.hood;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
// import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class HoodIOTalonFXS implements HoodIO {
	protected final TalonFXS motor = HardwareDevices.hoodMotorID.talonFXS();

	private final EncodedMotorStatusSignalCache motorStatusSignalCache;
    private final StatusSignal<ReverseLimitValue> limitSwitchStatusSignal;

    private final BaseStatusSignal[] refreshSignals;
    private final BaseStatusSignal[] motorConnectionSignals;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();
    
    public HoodIOTalonFXS() {
        var motorConfig = new TalonFXSConfiguration();

        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(HoodConstants.motorToMechanism.inverse().applyUnsigned(HoodConstants.maxAngle))
        ;
        // motorConfig.HardwareLimitSwitch
            // .withReverseLimitEnable(true)
            // .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS1)
            // .withReverseLimitRemoteSensorID(HardwareDevices.candiID.id)
            // .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            // .withForwardLimitEnable(false)
        // ;

        this.motor.getConfigurator().apply(motorConfig);

        this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
        this.limitSwitchStatusSignal = this.motor.getReverseLimit();

        this.refreshSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.encoder().position(),
            this.motorStatusSignalCache.encoder().velocity(),
            this.motorStatusSignalCache.motor().appliedVoltage(),
            this.motorStatusSignalCache.motor().statorCurrent(),
            this.motorStatusSignalCache.motor().supplyCurrent(),
            this.motorStatusSignalCache.motor().torqueCurrent(),
            this.motorStatusSignalCache.motor().deviceTemperature(),
            this.limitSwitchStatusSignal
        };
        this.motorConnectionSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.encoder().position(),
            this.motorStatusSignalCache.encoder().velocity(),
            this.motorStatusSignalCache.motor().appliedVoltage(),
            this.motorStatusSignalCache.motor().statorCurrent(),
            this.motorStatusSignalCache.motor().supplyCurrent(),
            this.motorStatusSignalCache.motor().torqueCurrent(),
            this.motorStatusSignalCache.motor().deviceTemperature(),
            this.limitSwitchStatusSignal
        };

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.limitSwitchStatusSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.motorStatusSignalCache.motor().getStatusSignals());
        // BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        // BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        this.motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.refreshSignals);
        inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectionSignals);
        inputs.motor.updateFrom(this.motorStatusSignalCache);
        inputs.limitSwitch = this.limitSwitchStatusSignal.getValue() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
            .withOutput(volts)
        );
    }

    @Override
    public void setPositionRads(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
        this.motor.setControl(this.positionRequest
            .withPosition(Units.radiansToRotations(positionRads))
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedforwardVolts)
        );
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
        this.motor.setControl(controlRequest);
    }

    @Override
    public void configTunables(PIDConstants pidConstants, double maxVelocity, double kA, double kV, double kG, double cruiseVelocity) {
        var config = new Slot0Configs();
		var motionMagicConfig = new MotionMagicConfigs();
        this.motor.getConfigurator().refresh(config);
		this.motor.getConfigurator().refresh(motionMagicConfig);
        pidConstants.update(config);
		motionMagicConfig.MotionMagicExpo_kA = kA;
		motionMagicConfig.MotionMagicExpo_kV = kV;
		motionMagicConfig.MotionMagicCruiseVelocity = cruiseVelocity;
		config.GravityType = GravityTypeValue.Arm_Cosine;
		config.kG = kG;
        this.motor.getConfigurator().apply(config);
		this.motor.getConfigurator().apply(motionMagicConfig);
    }

    @Override
    public void resetMotorPositionRads(double positionRads) {
        this.motor.setPosition(Units.radiansToRotations(positionRads));
    }
}

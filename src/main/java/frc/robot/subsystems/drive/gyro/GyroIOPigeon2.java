package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.robot.subsystems.drive.odometry.OdometryThread.Buffer;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon = HardwareDevices.pigeonID.pigeon2();

	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] connectedSignals;
	private final StatusSignal<Double> quatWSignal;
	private final StatusSignal<Double> quatXSignal;
	private final StatusSignal<Double> quatYSignal;
	private final StatusSignal<Double> quatZSignal;
	private final StatusSignal<AngularVelocity> yawVelocitySignal;
	private final StatusSignal<AngularVelocity> pitchVelocitySignal;
	private final StatusSignal<AngularVelocity> rollVelocitySignal;

	private final Buffer<Rotation3d> quatBuffer;

	public GyroIOPigeon2() {
		var config = new Pigeon2Configuration();
		config.MountPose
			.withMountPoseYaw(Degrees.of(-179.59326171875))
			.withMountPosePitch(Degrees.of(-0.29825273156166077))
			.withMountPoseRoll(Degrees.of(-0.2136882245540619))
		;
		this.pigeon.getConfigurator().apply(config);

		this.quatWSignal = this.pigeon.getQuatW();
		this.quatXSignal = this.pigeon.getQuatX();
		this.quatYSignal = this.pigeon.getQuatY();
		this.quatZSignal = this.pigeon.getQuatZ();
		this.yawVelocitySignal = this.pigeon.getAngularVelocityZWorld();
		this.pitchVelocitySignal = this.pigeon.getAngularVelocityYWorld();
		this.rollVelocitySignal = this.pigeon.getAngularVelocityXWorld();

		this.refreshSignals = new BaseStatusSignal[] {
			this.yawVelocitySignal,
			this.pitchVelocitySignal,
			this.rollVelocitySignal,
		};
		this.connectedSignals = new BaseStatusSignal[] {
			this.quatWSignal,
			this.quatXSignal,
			this.quatYSignal,
			this.quatZSignal,
			this.yawVelocitySignal,
			this.pitchVelocitySignal,
			this.rollVelocitySignal,
		};

		BaseStatusSignal.setUpdateFrequencyForAll(
			DriveConstants.odometryLoopFrequency,
			this.quatWSignal,
			this.quatXSignal,
			this.quatYSignal,
			this.quatZSignal
		);
		BaseStatusSignal.setUpdateFrequencyForAll(
			RobotConstants.rioUpdateFrequency,
			this.yawVelocitySignal,
			this.pitchVelocitySignal,
			this.rollVelocitySignal
		);

		this.quatBuffer = OdometryThread.getInstance().registerPhoenixComboSignal(
			(array) -> new Rotation3d(
				new Quaternion(
					array[0],
					array[1],
					array[2],
					array[3]
				)
			),
			Rotation3d[]::new,
			this.quatWSignal,
			this.quatXSignal,
			this.quatYSignal,
			this.quatZSignal
		);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.connected = BaseStatusSignal.isAllGood(this.connectedSignals);

		inputs.odometrySampleCount = this.quatBuffer.getSize();
		System.arraycopy(this.quatBuffer.getInternalBuffer(), 0, inputs.odometryGyroRotation, 0, inputs.odometrySampleCount);
		this.quatBuffer.clear();

		inputs.yawVelocityRadsPerSec = Units.degreesToRadians(this.yawVelocitySignal.getValueAsDouble());
		inputs.pitchVelocityRadsPerSec = Units.degreesToRadians(this.pitchVelocitySignal.getValueAsDouble());
		inputs.rollVelocityRadsPerSec = Units.degreesToRadians(this.rollVelocitySignal.getValueAsDouble());
	}

	@Override
	public void resetYawRads(double yawRads) {
		this.pigeon.setYaw(Units.radiansToDegrees(yawRads));
	}
}

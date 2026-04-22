package frc.robot.subsystems.shooter.aiming;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.aiming.passing.PassingCalc;
import frc.robot.subsystems.shooter.aiming.shooting.ShootingCalc;
import lombok.Getter;

public class AimingSystem extends SubsystemBase {
	public final ShootingCalc shootingCalc;
	public final PassingCalc passingCalc;

	@Getter
	private boolean autoFeedEnabled = false;
	@Getter
	private boolean autoFeedIgnoreTags = false;

	public AimingSystem(ShootingCalc shootingCalc, PassingCalc passingCalc) {
		super("Shooter/Aiming");

		this.shootingCalc = shootingCalc;
		this.passingCalc = passingCalc;
	}

	private void setAutoFeedEnabled(boolean enabled) {
		this.autoFeedEnabled = enabled;
		Logger.recordOutput("Subsystems/Shooter/Aiming/Auto Feed/Enabled", this.autoFeedEnabled);
	}

	private void setAutoFeedIgnoreTags(boolean enabled) {
		this.autoFeedIgnoreTags = enabled;
		Logger.recordOutput("Subsystems/Shooter/Aiming/Auto Feed/Ignore Tags", this.autoFeedIgnoreTags);
	}

	public Command aimAtHub(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Translation3d> aimPointSupplier, boolean autoFeedEnabled, boolean autoFeedIgnoreTags) {
		final var aimingSystem = this;
		return new Command() {
			{
				this.setName("Aim at Hub");
				this.addRequirements(aimingSystem);
			}

			@Override
			public void initialize() {
				aimingSystem.shootingCalc.calculate(
					robotPoseSupplier.get(),
					fieldSpeedsSupplier.get(),
					aimPointSupplier.get()
				);
				aimingSystem.setAutoFeedEnabled(autoFeedEnabled);
				aimingSystem.setAutoFeedIgnoreTags(autoFeedIgnoreTags);
			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	public Command aimToPass(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Translation3d> aimPointSupplier, boolean autoFeedEnabled, boolean autoFeedIgnoreTags) {
		final var aimingSystem = this;
		return new Command() {
			{
				this.setName("Aim to Pass");
				this.addRequirements(aimingSystem);
			}

			@Override
			public void initialize() {
				aimingSystem.passingCalc.calculate(
					robotPoseSupplier.get(),
					fieldSpeedsSupplier.get(),
					aimPointSupplier.get()
				);
				aimingSystem.setAutoFeedEnabled(autoFeedEnabled);
				aimingSystem.setAutoFeedIgnoreTags(autoFeedIgnoreTags);
			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}
}

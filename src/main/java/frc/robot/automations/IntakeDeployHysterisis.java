package frc.robot.automations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.intake.slam.IntakeSlamConstants;
import frc.util.EdgeDetector;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class IntakeDeployHysterisis implements Runnable {
    private final IntakeSlam intakeSlam;
    private final Command intakeDeployCommand;

    private static final LoggedTunable<Angle> hysterisisThreshold = LoggedTunable.from("Automations/Intake Deploy Hysterisis/Threshold", Degrees::of, IntakeSlamConstants.minAngle.plus(IntakeSlamConstants.maxAngle).div(2.0).in(Degrees));

    private final EdgeDetector teleopEnableEdgeDetector = new EdgeDetector(false);

    public IntakeDeployHysterisis(IntakeSlam intakeSlam, Command intakeDeployCommand) {
        this.intakeSlam = intakeSlam;
        this.intakeDeployCommand = intakeDeployCommand;
    }

    @Override
    public void run() {
        this.teleopEnableEdgeDetector.update(DriverStation.isTeleopEnabled());
        if (this.teleopEnableEdgeDetector.risingEdge() && this.intakeSlam.getMeasuredAngleRads() <= IntakeDeployHysterisis.hysterisisThreshold.get().in(Radians)) {
            CommandScheduler.getInstance().schedule(this.intakeDeployCommand);
        }
    }
}

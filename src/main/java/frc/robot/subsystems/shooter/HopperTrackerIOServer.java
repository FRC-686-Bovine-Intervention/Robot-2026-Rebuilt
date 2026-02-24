package frc.robot.subsystems.shooter;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class HopperTrackerIOServer implements HopperTrackerIO {
    private static final LoggedTunableNumber deployedHeightToBallCount = LoggedTunable.from("HopperTracker/HeightToBalls", 40/14.75);
    private static final LoggedTunableNumber retractedHeightToBallCount = LoggedTunable.from("HopperTracker/HeightToBalls", 35/24.75);

    private static final String toRobotTable = "/DriverDashboard/ToRobot";
    private static final String toDashboardTable = "/DriverDashboard/ToDashboard";

    private static final String hopperHeightName = "HopperHeight";
    private static final String intakeDeployedName = "IntakeDeployed";
    
    private final DoubleSubscriber hopperHeightSubscriber;

    private final DoublePublisher hopperHeightPublisher;
    private final BooleanPublisher intakeDeployedPublisher;

    private boolean intakeDeployed = true;

    public HopperTrackerIOServer() {
        System.out.println("[Init] Creating HopperTrackerIOServer");
    
        WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/hopper_tracker");

        var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);

        hopperHeightSubscriber =
            inputTable
                .getDoubleTopic(hopperHeightName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        
                var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);

        
        intakeDeployedPublisher = outputTable.getBooleanTopic(intakeDeployedName).publish();
        hopperHeightPublisher = outputTable.getDoubleTopic(hopperHeightName).publish();
    }

    @Override
    public void updateInputs(HopperTrackerIOInputs inputs) {
        if (hopperHeightSubscriber.readQueue().length > 0) {
            inputs.ballCount = (int) ((double) hopperHeightSubscriber.get() * (intakeDeployed ? deployedHeightToBallCount.getAsDouble() : retractedHeightToBallCount.getAsDouble()));
        }
    }

    @Override
    public void setIntakeDeployed(boolean deployed) {
        this.intakeDeployed = deployed;
        intakeDeployedPublisher.set(deployed);
    }
    
    @Override
    public void setBallCount(int balls) {
        hopperHeightPublisher.set(balls / (intakeDeployed ? deployedHeightToBallCount.getAsDouble() : retractedHeightToBallCount.getAsDouble()));
    }
}

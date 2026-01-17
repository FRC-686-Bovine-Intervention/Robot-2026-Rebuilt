package frc.robot.subsystems.rollers.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        super("Rollers/Indexer");
        this.io = io;
    }
}

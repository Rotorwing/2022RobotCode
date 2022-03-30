package frc.robot.commands;
import frc.robot.subsystems.Sweeper;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SweepIn  extends CommandBase {
    private final Sweeper m_sweeper;

    public SweepIn(Sweeper subsystem) {
        m_sweeper = subsystem;
        addRequirements(m_sweeper);
    }

    public void execute() {
        m_sweeper.run();
    }

    public void cancel(){
        m_sweeper.stop();
    }

    public boolean isFinished(){
        return true;
    }
}

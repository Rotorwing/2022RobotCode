package frc.robot.commands;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Sweeper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot  extends CommandBase {
    private final Storage m_storage;
    private final Sweeper m_sweeper;
    private final boolean stops;

    private final Timer timer;
    private boolean started = false;

    public Shoot(Storage storage, Sweeper sweeper, boolean stops) {
        m_storage = storage;
        m_sweeper = sweeper;
        this.stops = stops;
        addRequirements(m_storage);
        addRequirements(m_sweeper);
        timer = new Timer();
        timer.stop();
        timer.reset();
    }
    public Shoot(Storage storage, Sweeper sweeper) {
        this(storage, sweeper, false);
    }

    public void execute() {
        if (!started){
            timer.start();
        }
        m_storage.runIn();
        m_sweeper.run();
        started = true;
    }

    public void cancel(){
        m_storage.stop();
        m_sweeper.stop();
        timer.stop();
        timer.reset();
        started = false;
    }

    public boolean isFinished(){
        if (stops){
            if(timer.get() > 2){
                started = false;
                timer.stop();
                return true;
            }
            return false;
        }
        return true;
    }
}

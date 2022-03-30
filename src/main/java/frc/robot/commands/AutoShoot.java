package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShoot extends CommandBase {

    private final AutoAim aim;
    private final Shoot shoot;

    private final Shooter shooter;
    private final Storage storage;
    private Timer timer;

    public AutoShoot(Sweeper sweeper, Shooter shooter, Storage storage, Drive drive) {
        this.shooter = shooter;
        this.storage = storage;
        addRequirements(sweeper);
        addRequirements(shooter);
        addRequirements(storage);

        aim = new AutoAim(shooter, drive::getPose, drive::getRotation2d, false);
        shoot = new Shoot(storage, sweeper, true);
        System.out.println("Starting auto Shoot");

        timer = new Timer();
        timer.reset();
        timer.start();
    }

    public void execute() {
        //System.out.println("executing auto Shoot");
        aim.execute();
        //if (shooter.onTarget()){System.out.println("on Target");}
        if (shooter.onTarget() && timer.get() > 2){
            //System.out.println("Shooting");
            shoot.execute();
        }else{
            storage.stop();
        }
    }

    public void cancel(){
        storage.stop();
        timer.stop();
        timer.reset();
    }

    public boolean isFinished(){
        if(shoot.isFinished()){
            System.out.println("Done with Autoshoot");
            aim.cancel();
            storage.stop();
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }
}

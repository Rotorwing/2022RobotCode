package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;

public class ManualAim extends CommandBase {
    private final Shooter m_shooter;
    private final Joystick m_joystick;


    public ManualAim(Shooter subsystem, Joystick joystick) {
        m_shooter = subsystem;
        addRequirements(m_shooter);

        m_joystick = joystick;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //m_shooter.enable();
        //m_shooter.disable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setTarget(m_joystick.getX()*360); // m_shooter.rangeMap(m_joystick.getX()));
        //m_shooter.setTuretPct(m_joystick.getX());
        //m_shooter.setTuretPct(m_joystick.getX());
        m_shooter.setShooterPct((-m_joystick.getThrottle()*0.5+0.5)*2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}

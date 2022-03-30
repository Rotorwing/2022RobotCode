package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Joystick;


public class DefaultDrive extends CommandBase {

    private final Drive m_drive;
    private final Joystick m_joystick;


    public DefaultDrive(Drive subsystem, Joystick joystick) {

        m_drive = subsystem;
        addRequirements(m_drive);

        m_joystick = joystick;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double mult = 0.8;
        m_drive.arcadeDrive(m_joystick.getX()*mult, -m_joystick.getY()*mult);
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

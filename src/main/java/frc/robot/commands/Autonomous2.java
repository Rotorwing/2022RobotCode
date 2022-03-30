package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Autonomous2 extends SequentialCommandGroup {
    
    public Autonomous2(Sweeper m_sweeper, Shooter m_shooter, Storage m_storage, Drive m_drive, RamseteCommand ramseteCommand, Arm m_arm){
        // andThen(new RunCommand(() -> m_drive.tankDriveVolts(0, 0), m_drive));
        // andThen(new AutoShoot(m_sweeper, m_shooter, m_storage, m_drive));
        // andThen(new RunCommand(m_arm::down, m_arm));
        // andThen(new RunCommand(m_sweeper::run, m_sweeper));
        // andThen(ramseteCommand);
        // andThen(new RunCommand(() -> m_drive.tankDriveVolts(0, 0), m_drive));
        // andThen(new AutoShoot(m_sweeper, m_shooter, m_storage, m_drive));
        m_drive.setDefaultCommand(new RunCommand(m_drive::stop, m_drive));

        addCommands(
            new InstantCommand(m_arm::down, m_arm),
            new InstantCommand(m_sweeper::run, m_sweeper),
            new AutoShoot(m_sweeper, m_shooter, m_storage, m_drive),
            ramseteCommand,
            new InstantCommand(m_drive::stop, m_drive),
            new AutoShoot(m_sweeper, m_shooter, m_storage, m_drive)
        );
    }
}

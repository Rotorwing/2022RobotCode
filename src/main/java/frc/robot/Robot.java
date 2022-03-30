package frc.robot;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/*
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
*/
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.Position;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private Drive drive;
    private Shooter shooter;
    private Sweeper sweeper;
    private Storage storage;
    private Arm arm;
    private Climb climb;

    private Joystick driveStick;
    private Joystick auxtick;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        drive = m_robotContainer.m_drive;
        shooter = m_robotContainer.m_shooter;
        driveStick = m_robotContainer.driveJoystick.joystick;
        auxtick = m_robotContainer.auxJoystick.joystick;
        sweeper = m_robotContainer.m_sweeper;
        storage = m_robotContainer.m_storage;
        arm = m_robotContainer.m_arm;
        climb = m_robotContainer.m_climb;

        //HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

        m_robotContainer.auxJoystick.shootButton.whileHeld(new Shoot(storage, sweeper));

        m_robotContainer.driveJoystick.sweepButton.whileHeld(new SweepIn(sweeper));

        m_robotContainer.driveJoystick.armUp.whenPressed(new InstantCommand(arm::up, arm));
        m_robotContainer.driveJoystick.armDown.whenPressed(new InstantCommand(arm::down, arm));

        m_robotContainer.auxJoystick.climbUp.whenPressed(new InstantCommand(climb::extend, climb));
        m_robotContainer.auxJoystick.climbDown.whenPressed(new InstantCommand(climb::pull, climb));

        m_robotContainer.auxJoystick.climbUp.whenReleased(new InstantCommand(climb::stop, climb));
        m_robotContainer.auxJoystick.climbDown.whenReleased(new InstantCommand(climb::stop, climb));

        m_robotContainer.driveJoystick.backSweep.whileHeld(new RunCommand(sweeper::backFeed, sweeper));
        m_robotContainer.auxJoystick.backShoot.whileHeld(new RunCommand(() -> {shooter.setShooterPct(-0.2);}, shooter));

        m_robotContainer.auxJoystick.autoShoot.whileHeld(new AutoAim(shooter, drive::getPose, drive::getRotation2d));

        m_robotContainer.auxJoystick.setAtParked.whenPressed(new InstantCommand(() ->
            drive.resetOdometry(new Pose2d(-0.75, -4.4784, new Rotation2d(90)))
        , drive));

        SmartDashboard.putNumber("ShotKp", 1.6);
        SmartDashboard.putNumber("ShotkF",  1.8);

    }

    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

    }


    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit() {
        drive.arcadeDrive(0, 0);
        shooter.disable();
        arm.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit() {
        Path AutoPath = new Path(
            new Pose2d(0, -2.3, new Rotation2d(0)),
            new ArrayList<>(),
            new Pose2d(2.3, -3.4, new Rotation2d(-20))
        );
        Path Auto1Path = new Path(
            new Pose2d(-2.2, 1.14, new Rotation2d(0)),
            new ArrayList<>(),
            new Pose2d(-3.34, 1.17, new Rotation2d(0))
        );
                                  
        m_autonomousCommand = new Autonomous(sweeper, shooter, storage, drive, m_robotContainer.getAutonomousCommand(AutoPath), arm);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            //System.out.println("schedule command");
            m_autonomousCommand.schedule();
        }
        shooter.enable();
        storage.stop();

        CommandScheduler.getInstance().setDefaultCommand(drive, new RunCommand(drive::stop, drive));
        drive.setDefaultCommand(new RunCommand(drive::stop, drive));
    }

    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        //drive.resetOdometry(new Pose2d());
        shooter.enable();
        //drive.resetOdometry(new Pose2d(0, -2.5, new Rotation2d(0)));

        drive.setDefaultCommand(new DefaultDrive(drive, driveStick));
        
        shooter.setDefaultCommand(new ManualAim(shooter, auxtick));
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //m_drive.arcadeDrive(driveStick.getX(), driveStick.getY());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }

}
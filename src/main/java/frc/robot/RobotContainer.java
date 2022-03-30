package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

private static RobotContainer m_robotContainer = new RobotContainer();

// The robot's subsystems
    public final Shooter m_shooter = new Shooter();
    public final Drive m_drive = new Drive();
    public final Sweeper m_sweeper = new Sweeper();
    public final Storage m_storage = new Storage();
    public final Arm m_arm = new Arm();
    public final Climb m_climb = new Climb();

// Joysticks
  public class DriveJoystick {
    public final Joystick joystick = new Joystick(0);
    public final JoystickButton sweepButton = new JoystickButton(joystick, 2);
    public final JoystickButton armUp = new JoystickButton(joystick, 10); 
    public final JoystickButton armDown = new JoystickButton(joystick, 9); 
    public final JoystickButton backSweep = new JoystickButton(joystick, 7); 
  }
  public final DriveJoystick driveJoystick;

  public class AuxJoystick {
    public final Joystick joystick = new Joystick(1);
    public final JoystickButton shootButton = new JoystickButton(joystick, 1);
    public final JoystickButton climbUp = new JoystickButton(joystick, 10); 
    public final JoystickButton climbDown = new JoystickButton(joystick, 9); 
    public final JoystickButton backShoot = new JoystickButton(joystick, 7); 
    public final JoystickButton autoShoot = new JoystickButton(joystick, 2); 
    public final JoystickButton setAtParked = new JoystickButton(joystick, 3); 
  }
  public final AuxJoystick auxJoystick;

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    // Init joystick containers
    driveJoystick = new DriveJoystick();
    auxJoystick = new AuxJoystick();
    
    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("AutoAim", new AutoAim(m_shooter, m_drive::getPose, m_drive::getRotation2d));
    SmartDashboard.putData("ManualAim", new ManualAim(m_shooter, auxJoystick.joystick));
    SmartDashboard.putData("DefaultDrive", new DefaultDrive( m_drive, driveJoystick.joystick));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
// Create some buttons
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public RamseteCommand getAutonomousCommand(Path path) {
  // The selected command will be run in autonomous
  //return m_chooser.getSelected();

  // Create a voltage constraint to ensure we don't accelerate too fast
  var autoVoltageConstraint =
                              new DifferentialDriveVoltageConstraint(
                              new SimpleMotorFeedforward(
                                DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                              DriveConstants.kDriveKinematics,
                            5);

  // Create config for trajectory
  TrajectoryConfig config =
                            new TrajectoryConfig(
                            AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // Add kinematics to ensure max speed is actually obeyed
  .setKinematics(DriveConstants.kDriveKinematics)
  // Apply the voltage constraint
  .addConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  //System.out.println(path.getStart());
  //System.out.println(path.getWaypoints());
  //System.out.println(path.getEnd());
  Trajectory exampleTrajectory =
                                  TrajectoryGenerator.generateTrajectory(
                                  path.getStart(),
                                  path.getWaypoints(),
                                  path.getEnd(),
                                  config);

  

  RamseteCommand ramseteCommand =
                                  new RamseteCommand(
                                  exampleTrajectory,
                                  m_drive::getPose,
                                  new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                  new SimpleMotorFeedforward(
                                    DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                                  DriveConstants.kDriveKinematics,
                                  m_drive::getWheelSpeeds,
                                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                  // RamseteCommand passes volts to the callback
                                  m_drive::tankDriveVolts,
                                  m_drive);

  // Reset odometry to the starting pose of the trajectory.
  m_drive.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  return ramseteCommand;
  }
}


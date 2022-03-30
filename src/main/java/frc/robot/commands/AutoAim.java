package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.concurrent.Callable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.Shooter;

import java.lang.Math;

public class AutoAim extends CommandBase {

    private final Shooter m_shooter;
    private Callable<Pose2d> getOdometry;
    private Callable<Rotation2d> getRotation;
    private Pose2d currentPose;
    private double currentAngle;
    private Pose2d futurePose;


    boolean finishes = false;

    public AutoAim(Shooter subsystem, Callable<Pose2d> _getOdometry, Callable<Rotation2d> _getRotation, boolean ends) {
        finishes = ends;
        m_shooter = subsystem;
        getOdometry = _getOdometry;
        getRotation = _getRotation;
        addRequirements(m_shooter);
        
    }
    public AutoAim(Shooter subsystem, Callable<Pose2d> _getOdometry, Callable<Rotation2d> _getRotation) {
        this(subsystem, _getOdometry, _getRotation, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        try {
            currentPose = null;
            currentAngle = 0;
			currentPose = getOdometry.call();
            currentAngle = getRotation.call().getDegrees();
		} catch (Exception e) {
            DriverStation.reportError("Unable to get Odometry data in Shooter thread.", true);
			//e.printStackTrace();
		}
        /*if (currentPose != null){
            double[] rawPose = SmartDashboard.getNumberArray("PoseResult", new double[0]);
            Pose2d updatePose = new Pose2d(rawPose[0], rawPose[1], new Rotation2d(rawPose[1]));
            futurePose = new Pose2d(currentPose.getX()+updatePose.getX(), currentPose.getY()+updatePose.getY(),
                                    new Rotation2d (currentPose.getRotation().getDegrees()+updatePose.getRotation().getDegrees()));
        }*/
        currentPose = new Pose2d(currentPose.getX()-Math.cos(Math.toRadians(currentAngle))*0.18, currentPose.getY()-Math.sin(Math.toRadians(currentAngle))*0.18, currentPose.getRotation());

        double tAngle = 0;
        double distance = 0;
        tAngle = (Math.toDegrees(-Math.atan2(currentPose.getY(), currentPose.getX()))+currentAngle+180)%360;
        distance = Math.hypot(currentPose.getX(), currentPose.getY());
        if (distance > 10){
            distance = 0;
        }
        SmartDashboard.putNumber("TargetAngle", tAngle);
        SmartDashboard.putNumber("TargetDistance", distance);

        m_shooter.setSetpoint(tAngle);
        //1.6 1.8
        m_shooter.setShooterVolts(distance*SmartDashboard.getNumber("ShotKp", 0)+SmartDashboard.getNumber("ShotkF", 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (finishes){
            m_shooter.onTarget();
        }
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }

    public double[] computePowers(double distance){
        double[] output = {0, 0};

        output[0] = distance*0.7;
        output[1] = distance*0.1;

        return output;
    }
}

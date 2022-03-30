package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;


public class Drive extends SubsystemBase {
    // private PWMVictorSPX bLDrive;
    // private PWMVictorSPX fLDrive;
    // private PWMVictorSPX bRDrive;
    // private PWMVictorSPX fRDrive;
    private CANSparkMax bLDrive;
    private CANSparkMax fLDrive;
    private CANSparkMax bRDrive;
    private CANSparkMax fRDrive;
    private MotorControllerGroup leftDrive;
    private MotorControllerGroup rightDrive;
    private DifferentialDrive differentialDrive;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private ADXRS450_Gyro gyro;
    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private DifferentialDriveOdometry odometry;

    private double lastLeft = 0;
    private double lastRight = 0;
    private double lastAngle = 0;

    private Timer recordingClock = new Timer();
    private double lastRecord = 0;

    private RecordingPoint[] RecordData;

    private double startAngle = 0;

    private class RecordingPoint{
        public double deltaLeft;
        public double deltaRight;
        public double deltaAngle;
        public double joystickLeft;
        public double joystickRight;

        public RecordingPoint(double dleft, double dright, double dangle, double joyleft, double joyright){
            deltaLeft = dleft;
            deltaRight = dright;
            deltaAngle = dangle;
            joystickLeft = joyleft;
            joystickRight = joyright;
        }
        public RecordingPoint(){
            this(0.0, 0.0, 0.0, 0.0, 0.0);
        }
    }

    public Drive() {

        bLDrive = new CANSparkMax(DriveConstants.kLeftMotor2Id, MotorType.kBrushless);
        bLDrive.setInverted(false);

        fLDrive = new CANSparkMax(DriveConstants.kLeftMotor1Id, MotorType.kBrushless);
        fLDrive.setInverted(false);

        bRDrive = new CANSparkMax(DriveConstants.kRightMotor2Id, MotorType.kBrushless);
        bRDrive.setInverted(false);

        fRDrive = new CANSparkMax(DriveConstants.kRightMotor1Id, MotorType.kBrushless);
        fRDrive.setInverted(false);

        rightDrive = new MotorControllerGroup(fRDrive, bRDrive  );
        addChild("RightDrive",rightDrive);

        leftDrive = new MotorControllerGroup(fLDrive, bLDrive  );
        addChild("LeftDrive",leftDrive);


        differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
        addChild("DifferentialDrive",differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        odometry = new DifferentialDriveOdometry(new Rotation2d(0));

        leftEncoder = fLDrive.getEncoder();
        rightEncoder = fRDrive.getEncoder();

        gyro = new ADXRS450_Gyro();
        gyro.reset();
        gyro.calibrate();

        navX.reset();
        navX.calibrate();

        System.out.println("Gyro: "+gyro.isConnected()+"prot"+ gyro.getPort());

        RecordData = new RecordingPoint[10];
        Arrays.fill(RecordData, new RecordingPoint());
        recordingClock.reset();
        recordingClock.start();
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    private RecordingPoint makeDatapoint(){
        return new RecordingPoint(getDeltaLeftOdo(), getDeltaRightOdo(), getDeltaAngle(), leftDrive.get(), rightDrive.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        updateOdometry();
        SmartDashboard.putNumber("OdoX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("OdoY", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("EncoderL", getLeftOdo());
        SmartDashboard.putNumber("EncoderR", getRightOdo());
        SmartDashboard.putNumber("EncoderLV", leftEncoder.getVelocity()*DriveConstants.kEncoderMultiplier);
        SmartDashboard.putNumber("EncoderRV", -rightEncoder.getVelocity()*DriveConstants.kEncoderMultiplier);

        // This method will be called once per scheduler run
        double cClock = recordingClock.get();
        if ( cClock - lastRecord >= 0.1 ){
            lastRecord = cClock;
            for (int i = 0; i < RecordData.length-1; i++){
                RecordData[i] = RecordData[i+1];
            }
            RecordData[RecordData.length-1] = makeDatapoint();
            double[] output  = new double[5*10];

            for (int i = 0; i < RecordData.length-1; i++){
                int outIndex = i*5;
                output[outIndex] = RecordData[i].joystickLeft;
                output[outIndex+1] = RecordData[i].joystickRight;
                output[outIndex+2] = RecordData[i].deltaLeft;
                output[outIndex+3] = RecordData[i].deltaRight;
                output[outIndex+4] = RecordData[i].deltaAngle;
            }

            SmartDashboard.putNumberArray("RecordData", output);
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcadeDrive(double x, double y){
        differentialDrive.arcadeDrive(x, y);
    }

    public double getLeftOdo(){
        return leftEncoder.getPosition()*DriveConstants.kEncoderMultiplier;
    }

    public double getRightOdo(){
        return -rightEncoder.getPosition()*DriveConstants.kEncoderMultiplier;
    }
    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, pose.getRotation());
        //navX.setAngleAdjustment(pose.getRotation().getDegrees());
        //startAngle = pose.getRotation().getDegrees();
    }

    public double getAngle(){
        return navX.getRotation2d().getDegrees()+startAngle;
    }
    public Rotation2d getRotation2d(){
        return navX.getRotation2d();
    }

    private double getDeltaLeftOdo(){
        double val = leftEncoder.getPosition();
        double out = val-lastLeft;
        lastLeft = val;
        return out;
    }

    private double getDeltaRightOdo(){
        double val = rightEncoder.getPosition();
        double out = val-lastRight;
        lastRight = val;
        return out;
    }

    private double getDeltaAngle(){
        double val = getAngle();
        double out = val-lastAngle;
        lastAngle = val;
        return out; //navX.getRate();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()*DriveConstants.kEncoderMultiplier, -rightEncoder.getVelocity()*DriveConstants.kEncoderMultiplier);
    }

    public void updateOdometry(){
        //odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

        odometry.update(navX.getRotation2d(), getLeftOdo(), getRightOdo());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        //System.out.println(leftVolts+", "+rightVolts+" volts");
        leftDrive.setVoltage(Math.max(Math.min(leftVolts, 8), -8));
        rightDrive.setVoltage(-Math.max(Math.min(rightVolts, 8), -8));
        differentialDrive.feed();
    }
    public boolean stop(){
        tankDriveVolts(0, 0);
        return false;
    }

}


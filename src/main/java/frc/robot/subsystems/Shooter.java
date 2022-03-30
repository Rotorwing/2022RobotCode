package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import java.lang.Math;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {

    // public static final double k_maxAngle = 0;
    // public static final double k_minAngle = 360;

    private Encoder m_encoder;
    private VictorSPX hMotor;
    private DigitalInput m_limitSwitch;
    private CANSparkMax m_shooter1;
    private CANSparkMax m_shooter2;
    private VictorSPX m_omni;

    private SparkMaxPIDController m_shooter1PID;
    private SparkMaxPIDController m_shooter2PID;

    private RelativeEncoder m_shooter1Encoder;
    private RelativeEncoder m_shooter2Encoder;

    private double encoderOffset = 0.0;
    private double limitPosition = 0;
    private boolean calibrated = false;

    //P I D Variables
    private static final double kP = 0.07;
    private static final double kI = 0.002;
    private static final double kD = 0.00006;
    private static final double kF = 0.0;
    private static final double kTolerance = 9;

    private double lastPower = 0;

    // Initialize your subsystem here
    public Shooter() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(kTolerance);

        m_encoder = new Encoder(ShooterConstants.kEncoderData1, ShooterConstants.kEncoderData2, false, EncodingType.k4X);
        addChild("HEncoder",m_encoder);
        m_encoder.setDistancePerPulse(ShooterConstants.kDegreesPerPulse);

        hMotor = new VictorSPX(ShooterConstants.kTuretMotorId);
        hMotor.setInverted(true);

        m_limitSwitch = new DigitalInput(ShooterConstants.kLimitPort);

        m_shooter1 = new CANSparkMax(ShooterConstants.kShooterMotor1Id, MotorType.kBrushless);
        m_shooter2 = new CANSparkMax(ShooterConstants.kShooterMotor2Id, MotorType.kBrushless);
        m_omni = new VictorSPX(ShooterConstants.kOmniMotorId);

        m_shooter1.setInverted(true);
        m_shooter2.setInverted(false);
        m_omni.setInverted(true);

        m_shooter1PID = m_shooter1.getPIDController();
        m_shooter2PID = m_shooter2.getPIDController();

        m_shooter1Encoder = m_shooter1.getEncoder();
        m_shooter2Encoder = m_shooter2.getEncoder();

        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();
        SmartDashboard.putNumber("Shooter Wheel 1", m_shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter Wheel 2", m_shooter2Encoder.getVelocity());
        SmartDashboard.putNumber("Turret Angle", getAngle());

        if(getLimitSwitch()){
            encoderOffset = limitPosition-m_encoder.getDistance();
            calibrated = true;
        }
        if(lastPower > 0.9 || lastPower < -0.9){
            getController().reset();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public double getAngle(){
        return m_encoder.getDistance()+encoderOffset;
    }

    @Override
    public double getMeasurement() {
        return getAngle();
    }


    @Override
    public void useOutput(double output, double setpoint) {
        output += setpoint*kF;
        // if (getAngle() <= k_minAngle){
        //     output = Math.max(0, output);
        // }
        // if (getAngle() >= k_maxAngle){
        //     output = Math.min(0, output);
        // }

        //System.out.println(output);
        //output = Math.min(0.9, Math.max(-0.9, output));
        lastPower = output;
        hMotor.set(ControlMode.PercentOutput, output);
    }
    public void setTarget(double target) {
        //System.out.println((((target+180) % 360)+360)%360 -180);
        //target = target%360;// rangeLimit((((target+270) % 360)+360)%360 -270);
        setSetpoint(target);
    }
    public double rangeLimit(double input){
        return input; //Math.max(k_minAngle, Math.min(k_maxAngle, input));
    }
    public double rangeMap(double input){
        return input;
        // return Math.max(k_minAngle, Math.min(k_maxAngle, input*Math.max(k_maxAngle, -k_minAngle)));
    }

    public void setShooterWheels(double RPM1, double RPM2){
        m_shooter1PID.setReference(RPM1, CANSparkMax.ControlType.kVelocity);
        m_shooter2PID.setReference(RPM2, CANSparkMax.ControlType.kVelocity);
    }

    public boolean getLimitSwitch(){
        return m_limitSwitch.get();
    }

    public boolean isCalibrated(){
        return calibrated;
    }
    public void setShooterPct(double val){
        m_shooter1.set(val);
        m_shooter2.set(val);
        if (val > 0){
            startOmni();
        }
        else{
            stopOmni();
        }
    }
    public void setShooterVolts(double volts){
        m_shooter1.setVoltage(volts);
        m_shooter2.setVoltage(volts);
        if (volts > 0){
            startOmni();
        }
        else{
            stopOmni();
        }
    }
    public boolean onTarget(){
        return getController().atSetpoint() && (lastPower < 0.4 && lastPower > -0.4);
    }
    public void setTuretPct(double val){
        this.disable();
        hMotor.set(ControlMode.PercentOutput, val);
    }

    public void startOmni(){
        m_omni.set(ControlMode.PercentOutput, ShooterConstants.kOmniPower);
    }
    public void stopOmni(){
        m_omni.set(ControlMode.PercentOutput, 0);
        
    }
}

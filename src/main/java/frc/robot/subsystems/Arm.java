package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;

public class Arm extends SubsystemBase {
    private VictorSPX mainMotor;
    private DigitalInput m_lowerLimit;
    private DigitalInput m_upperLimit;

    public Arm() {

        mainMotor = new VictorSPX(ArmConstants.kMotorId);
        mainMotor.setInverted(true);
        m_lowerLimit = new DigitalInput(ArmConstants.kLowerLimitDIO);
        m_upperLimit = new DigitalInput(ArmConstants.kUpperLimitDIO);
    }

    public boolean getLowerLimit(){
        return ! m_lowerLimit.get();
    }
    public boolean getUpperLimit(){
        return ! m_upperLimit.get();
    }

    public void setPower(double val){
        if (getLowerLimit() && val < 0){
            val = 0;
        }else if (getUpperLimit() && val > 0){
            val = 0;
        }
        mainMotor.set(VictorSPXControlMode.PercentOutput, val);
    }

    public void up(){
        setPower(ArmConstants.kUpPower);
    }
    public void down(){
        setPower(ArmConstants.kDownPower);
    }
    public void stop(){
        setPower(0);
    }
}

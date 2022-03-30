package frc.robot.subsystems;

import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climb extends SubsystemBase {

    private PWMVictorSPX mainMotor;

    public Climb() {
        mainMotor = new PWMVictorSPX(ClimbConstants.kPWMId);
    }

    @Override
    public void periodic() {

    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    public void pull(){
        mainMotor.set(ClimbConstants.kPullPower);
    }
    public void stop(){
        mainMotor.set(0);
    }
    public void extend(){
        mainMotor.set(ClimbConstants.kExtendPower);
        mainMotor.stopMotor();
    }
    
}

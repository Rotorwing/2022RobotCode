package frc.robot.subsystems;

import frc.robot.Constants.SweeperConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;


public class Sweeper extends SubsystemBase {

    private VictorSPX mainMotor;

    public Sweeper() {
        mainMotor = new VictorSPX(SweeperConstants.kSweepMotorId);
    }

    @Override
    public void periodic() {

    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    public void run(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, SweeperConstants.kSweepSpeed);
    }
    public void stop(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    public void backFeed(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, 0.7);
    }
    
}

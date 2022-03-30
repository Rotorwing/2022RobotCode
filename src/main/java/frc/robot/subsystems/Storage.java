package frc.robot.subsystems;

import frc.robot.Constants.StorageConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;


public class Storage extends SubsystemBase {

    private VictorSPX mainMotor;

    public Storage() {
        mainMotor = new VictorSPX(StorageConstants.kStorageMotorId);
    }

    @Override
    public void periodic() {

    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    public void runIn(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, StorageConstants.kStorageSpeed);
    }
    public void runOut(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, -0.3);
    }
    public void stop(){
        mainMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
}

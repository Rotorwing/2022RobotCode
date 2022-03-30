package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final class DriveConstants {
        public static final int kLeftMotor1Id = 5;
        public static final int kLeftMotor2Id = 2;
        public static final int kRightMotor1Id = 3;
        public static final int kRightMotor2Id = 4; 

        public static final double ksVolts =  0.164; //0.18683; //0.1868;
        public static final double kvVoltSecondsPerMeter = 8.559;//2.7256; 
        public static final double kaVoltSecondsSquaredPerMeter = 1.7219; //0.29641; //1.5874; //;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.577);

        public static final double kPDriveVel = 0.030821; //0.000021403; //17.957; //3.3532;

        public static final double kEncoderMultiplier = 0.478778/10.71; // 0.217777777777778 // *2.244329896907216
    }
    public static final class AutoConstants {
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;
    }
    
    public static final class ShooterConstants {
        public static final int kShooterMotor1Id = 6;
        public static final int kShooterMotor2Id = 7;
        public static final int kOmniMotorId = 12;
        public static final int kTuretMotorId = 8;

        public static final int kEncoderData1 = 0;
        public static final int kEncoderData2 = 1;
        public static final int kLimitPort = 3;

        public static final double kDegreesPerPulse = -1.1;

        public static final double kOmniPower = 0.3;

        public static final class TuretPID{
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }

    public static final class SweeperConstants {
        public static final int kSweepMotorId = 9;
        public static final int kSweepSpeed = -1;
    }

    public static final class StorageConstants {
        public static final int kStorageMotorId = 10;
        public static final int kStorageSpeed = 1;
    }

    public static final class ArmConstants {
        public static final int kMotorId = 11;

        public static final int kLowerLimitDIO = 6;
        public static final int kUpperLimitDIO = 7;
        
        public static final double kDownPower = -0.25;
        public static final double kUpPower = 0.75;
    }

    public static final class ClimbConstants {
        public static final double kPullPower = 1;
        public static final double kExtendPower = -1;
        public static final int kPWMId = 0;
    }
}


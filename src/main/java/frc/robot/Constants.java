package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  // 駕駛員手把設定
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.05;
  }

  // 底盤 (AM14U: 4x NEO)
  public static class DriveConstants {
    public static final int kLeftMasterCanId = 1;
    public static final int kLeftFollowerCanId = 2;
    public static final int kRightMasterCanId = 3;
    public static final int kRightFollowerCanId = 4;

    public static final int kCurrentLimit = 40;          
    public static final double kWheelDiameterMeters = 0.1524;    
    public static final double kGearRatio = 8.45; 
    public static final double kPositionFactor = (1.0 / kGearRatio) * (Math.PI * kWheelDiameterMeters);
    public static final double kVelocityFactor = kPositionFactor / 60.0;
    public static final double kDriveSlewRate = 3.0;
  }

  // 視覺
  public static class VisionConstants {
    public static final double kCameraHeightMeters = Units.inchesToMeters(15.0);
    public static final double kTargetHeightMeters = Units.inchesToMeters(104.0);
    public static final double kCameraPitchRadians = Units.degreesToRadians(20.0);
  }

  // Intake (折疊: NEO 1:20, 驅動: Falcon)
  public static class IntakeConstants {
    public static final int kPivotCanId = 7;   // NEO
    public static final int kRollerCanId = 8;  // Falcon
    
    public static final double kIntakeSpeed = 0.6; 
    public static final double kreIntakeSpeed = -0.6; 

    // Pivot 齒輪比轉換 (讓 Encoder 讀數變成角度)
    public static final double kPivotGearRatio = 20.0;
    public static final double kPivotPositionFactor = 360.0 / kPivotGearRatio;
    
    public static final double kPivotKp = 0.03;
    public static final double kPivotKi = 0.0;
    public static final double kPivotKd = 0.0;
    public static final double kPivotKg = 0.2; 
    
    public static final double kPivotStowAngle = 0.0;   
    public static final double kPivotDeployAngle = 90.0; 
  }

  // Indexer & Feeder (履帶: Falcon, 輸球: NEO 1:4)
  public static class IndexerConstants {
    public static final int kConveyorCanId = 9;  // Falcon
    public static final int kFeederCanId = 10;   // NEO
    
    public static final double kConveyorSpeed = 0.5;
    public static final double kFeederSpeed = 0.8;
  }

  // Shooter (上 1 Falcon, 下 1 Falcon)
  public static class ShooterConstants {
    public static final int kTopCanId = 21; 
    public static final int kBottomCanId = 22;
    
    public static final double kP = 0.11; 
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12; 
    
    public static final double kStandardRPM = 2500.0; 
    public static final double kTrimSensitivity = 500.0; 
  }
}
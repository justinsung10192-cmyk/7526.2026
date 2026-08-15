package frc.robot;

public final class Constants {
  // 駕駛員手把設定
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.05; // 搖桿死區，避免漂移
  }

  // 底盤常數
  public static class DriveConstants {
    public static final int kLeftMasterCanId = 1;
    public static final int kLeftFollowerCanId = 2;
    public static final int kRightMasterCanId = 3;
    public static final int kRightFollowerCanId = 4;

    // 安全與性能設定
    public static final int kCurrentLimit = 40;          // 電流限制 (Amps)
    public static final double kWheelDiameterMeters = 0.1524;    // 6 英吋輪子 0.1524 公尺
    public static final double kGearRatio = 8.45; //齒輪比
    // 馬達轉一圈，機器人前進的距離 = (1 / 齒輪比) * (PI * 輪徑)
    public static final double kPositionFactor = (1.0 / kGearRatio) * (Math.PI * kWheelDiameterMeters);
    // RPM 轉換為 公尺/秒 = PositionFactor / 60
    public static final double kVelocityFactor = kPositionFactor / 60.0;
    public static final double kDriveSlewRate = 3.0;
    
  }

  // Intake 常數
  public static class IntakeConstants {
    public static final int kIntakeCanId = 7;
    public static final double kIntakeSpeed = 0.5; // 進件預設速度
    public static final double kreIntakeSpeed = -0.5; // 退件預設速度
    public static final double kEjectSpeed = -0.5; // 吐件預設速度
  }
  // 在 Constants.java 類別內部新增：

  public static class ShooterConstants {
    public static final int kShooterCanId = 21; // 請修改為你實際在 Phoenix Tuner 設定的 CAN ID
    public static final int kShooterFollowCanId = 11;
    public static final double kStandardSpeed = 0.7; // 標準射球預設速度 (0.0 ~ 1.0)
    public static final double kFarSpeed = 1.0;     // 遠距/回送模式預設速度
    public static final double kTrimSensitivity = 0.25; // 搖桿微調幅度 (加減 ±25% 速度)
  }

  
}


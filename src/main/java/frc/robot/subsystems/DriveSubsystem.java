// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import frc.robot.Constants.DriveConstants;
// import com.ctre.phoenix.motorcontrol.NeutralMode;


// public class DriveSubsystem extends SubsystemBase {
//   private final WPI_VictorSPX m_leftFront = new WPI_VictorSPX(DriveConstants.kLeftFrontCanId);
//   private final WPI_VictorSPX m_leftRear = new WPI_VictorSPX(DriveConstants.kLeftRearCanId);
//   private final WPI_VictorSPX m_rightFront = new WPI_VictorSPX(DriveConstants.kRightFrontCanId);
//   private final WPI_VictorSPX m_rightRear = new WPI_VictorSPX(DriveConstants.kRightRearCanId);

//   private final DifferentialDrive m_drive;

//   public DriveSubsystem() {
//     m_leftFront.configFactoryDefault();
//     m_leftRear.configFactoryDefault();
//     m_rightFront.configFactoryDefault();
//     m_rightRear.configFactoryDefault();

//     // 軸向跟隨 (Leader-Follower)
//     m_leftRear.follow(m_leftFront);
//     m_rightRear.follow(m_rightFront);

//     // 右側反轉
//     m_leftFront.setInverted(false);
//     m_rightFront.setInverted(true);

//     m_leftRear.setInverted(InvertType.FollowMaster);
//     m_rightRear.setInverted(InvertType.FollowMaster);

//     m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

//     setBrakeMode(true);
//   }

//   public void setBrakeMode(boolean enableBrake) {
//     NeutralMode mode = enableBrake ? NeutralMode.Brake : NeutralMode.Coast;
//     m_leftFront.setNeutralMode(mode);
//     m_leftRear.setNeutralMode(mode);
//     m_rightFront.setNeutralMode(mode);
//     m_rightRear.setNeutralMode(mode);
//   }
//   /** 回傳一個「以指定前進速度與轉向角速度持續開車，結束時自動停止」的 Command */
//   public Command runDriveCommand(double fwd, double rot) {
//     return this.runEnd(
//         () -> arcadeDrive(-fwd,- rot),
//         () -> stop()
//     );
//   }

//   /** Arcade 駕駛模式 (前進速度, 轉向角速度) */
//   public void arcadeDrive(double fwd, double rot) {
//     m_drive.arcadeDrive(-fwd, -rot);
//   }

//   public void stop() {
//     m_drive.stopMotor();
//   }
// }

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // 1. 宣告 4 顆 NEO 馬達 (SparkMax 控制器)
    private final SparkMax leftMaster = new SparkMax(DriveConstants.kLeftMasterCanId, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(DriveConstants.kLeftFollowerCanId, MotorType.kBrushless);
    private final SparkMax rightMaster = new SparkMax(DriveConstants.kRightMasterCanId, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(DriveConstants.kRightFollowerCanId, MotorType.kBrushless);

    // 2. 取得內建編碼器 (Encoders)
    private final RelativeEncoder leftEncoder = leftMaster.getEncoder();
    private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

    // 3. WPILib 差速驅動類別 (Arcade Drive)
    private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

    public DriveSubsystem() {
        // --- 設定左側馬達 ---
        SparkMaxConfig leftMasterConfig = new SparkMaxConfig();
        leftMasterConfig
            .idleMode(IdleMode.kBrake)                  // 煞車模式 (放開搖桿時立刻煞停)
            .smartCurrentLimit(DriveConstants.kCurrentLimit) // 電流限制
            .inverted(false);                           // 左側設為正向

        leftMasterConfig.encoder
            .positionConversionFactor(DriveConstants.kPositionFactor) // 轉換為「公尺」
            .velocityConversionFactor(DriveConstants.kVelocityFactor); // 轉換為「公尺/秒」

        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        leftFollowerConfig
            .apply(leftMasterConfig)                    // 複製主馬達設定
            .follow(leftMaster);                        // 跟隨左主馬達

        // --- 設定右側馬達 ---
        SparkMaxConfig rightMasterConfig = new SparkMaxConfig();
        rightMasterConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.kCurrentLimit)
            .inverted(true);                            // 右側通常需要反轉 (Inverted)

        rightMasterConfig.encoder
            .positionConversionFactor(DriveConstants.kPositionFactor)
            .velocityConversionFactor(DriveConstants.kVelocityFactor);

        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
        rightFollowerConfig
            .apply(rightMasterConfig)
            .follow(rightMaster);                       // 跟隨右主馬達

        // 套用設定到馬達控制器 (Reset 參數並持久化寫入 Flash)
        // 替代寫法（最推薦，簡短且不會產生歧義）
leftMaster.configure(leftMasterConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
leftFollower.configure(leftFollowerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
rightMaster.configure(rightMasterConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
rightFollower.configure(rightFollowerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // 重置編碼器數值
        resetEncoders();
    }

    @Override
    public void periodic() {
        // 即時在 Dashboard 上輸出底盤行駛距離與速度
        SmartDashboard.putNumber("Drive/Left Distance (m)", getLeftDistance());
        SmartDashboard.putNumber("Drive/Right Distance (m)", getRightDistance());
        SmartDashboard.putNumber("Drive/Average Speed (m/s)", getAverageVelocity());
    }

    /**
     * 手動駕駛控制 (Arcade Drive)
     * @param xSpeed 前進/後退速度 (-1.0 到 1.0)
     * @param zRotation 旋轉速度 (-1.0 到 1.0)
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * 專門提供給 PhotonVision 自動瞄準使用的轉向方法
     * @param xSpeed 前進速度
     * @param turnOutput PID 控制器算出的轉向輸出
     */
    public void autoAimDrive(double xSpeed, double turnOutput) {
        // 使用 squareInputs = false 避免 PID 輸出的線性度被二次方打亂
        drive.arcadeDrive(xSpeed, turnOutput, false);
    }

    /** 取得左側平均累積距離 (公尺) */
    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    /** 取得右側平均累積距離 (公尺) */
    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    /** 取得底盤平均速度 (公尺/秒) */
    public double getAverageVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
    }

    /** 清空編碼器數值 */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /** 煞車停止 */
    public void stop() {
        drive.stopMotor();
    }
}
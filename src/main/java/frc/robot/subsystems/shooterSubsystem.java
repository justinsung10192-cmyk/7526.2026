package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class shooterSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_mainMotor = new WPI_VictorSPX(ShooterConstants.kShooterCanId);
  private final WPI_VictorSPX m_followerMotor = new WPI_VictorSPX(ShooterConstants.kShooterFollowCanId);

  public shooterSubsystem() {
    m_mainMotor.configFactoryDefault();
    m_followerMotor.configFactoryDefault();

    // 設定副馬達跟隨主馬達
    m_followerMotor.follow(m_mainMotor);

    m_mainMotor.setInverted(false);

    // 如果兩顆滾輪是「對稱夾球（上下或左右對轉）」，副馬達選 OpposeMaster (反向)
    m_followerMotor.setInverted(InvertType.OpposeMaster);

    // 如果兩顆馬達是「同軸平行帶動」，改用這行：
    // m_followerMotor.setInverted(InvertType.FollowMaster);
  }

  public void setSpeed(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed));
    m_mainMotor.set(speed); // 主馬達一動，副馬達自動同步運轉！
  }

  public void stop() {
    m_mainMotor.set(0.0);
  }

  public Command runShooterCommand(double baseSpeed, DoubleSupplier stickInputSupplier) {
    return this.runEnd(
        () -> {
          double trim = -stickInputSupplier.getAsDouble() * ShooterConstants.kTrimSensitivity;
          double finalSpeed = baseSpeed + trim;
          setSpeed(finalSpeed);
        },
        () -> {
          stop();
        }
    );
  }

  public Command runShooterreCommand(double baseSpeed, DoubleSupplier stickInputSupplier) {
    return this.runEnd(
        () -> {
          double trim = -stickInputSupplier.getAsDouble() * ShooterConstants.kTrimSensitivity;
          double finalSpeed = baseSpeed + trim;
          setSpeed(-finalSpeed);
        },
        () -> {
          stop();
        }
    );
  }
}
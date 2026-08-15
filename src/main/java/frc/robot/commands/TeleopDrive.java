package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * TeleopDrive 指令負責將駕駛員的輸入轉換為底盤動作。
 * 這是 Command-Based 框架中「指令 (Command)」的具體實現。
 */
public class TeleopDrive extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_zRotation;

  // SlewRateLimiter 用於平滑加速和減速
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(DriveConstants.kDriveSlewRate);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(DriveConstants.kDriveSlewRate);

  /**
   * 建立 TeleopDrive 指令。
   *
   * @param drive 底盤子系統
   * @param xSpeed 前進速度來源 (Lambda 表達式)
   * @param zRotation 旋轉速度來源 (Lambda 表達式)
   */
  public TeleopDrive(DriveSubsystem drive, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    m_drive = drive;
    m_xSpeed = xSpeed;
    m_zRotation = zRotation;

    // 宣告此指令需要使用底盤子系統，防止多個指令同時操作底盤
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    // 從輸入來源獲取數值，應用死區，並透過 SlewRateLimiter 平滑處理後執行 Arcade Drive
    double xSpeed = m_xSpeed.getAsDouble();
    double zRotation = m_zRotation.getAsDouble();

    // 應用搖桿死區
    xSpeed = Math.abs(xSpeed) < OIConstants.kJoystickDeadband ? 0.0 : xSpeed;
    zRotation = Math.abs(zRotation) < OIConstants.kJoystickDeadband ? 0.0 : zRotation;

    m_drive.arcadeDrive(m_speedLimiter.calculate(xSpeed), m_rotationLimiter.calculate(zRotation));
  }

  @Override
  public void end(boolean interrupted) {
    // 指令結束或被中斷時停止底盤
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    // 此指令在手動操控階段永遠不會主動結束
    return false;
  }
}

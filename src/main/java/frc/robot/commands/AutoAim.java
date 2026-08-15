package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AutoAim extends Command {
    private final DriveSubsystem m_drive;
    private final Vision m_vision;

    // PID 控制器參數 (kP, kI, kD)
    // kP = 0.03 代表每偏離 1 度，輸出 0.03 (3%) 的轉向馬達力量
    // 提示：如果現場測試轉太慢可微調加大 kP (如 0.04)；若震盪晃動請減小 kP
    private final PIDController m_pid = new PIDController(0.03, 0.0, 0.002);

    public AutoAim(DriveSubsystem drive, Vision vision) {
        m_drive = drive;
        m_vision = vision;

        // 設定目標角度為 0 度（代表 AprilTag 正對相機中心）
        m_pid.setSetpoint(0.0);
        // 設定容許誤差為 ±1 度
        m_pid.setTolerance(1.0);

        // 必須宣告需要的 Subsystem，防止手動駕駛與自動對準同時搶奪底盤控制權
        addRequirements(m_drive, m_vision);
    }

    @Override
    public void initialize() {
        m_pid.reset();
    }

    @Override
    public void execute() {
        if (m_vision.hasTarget()) {
            double currentYaw = m_vision.getTargetYaw();
            
            // PIDController 計算轉向輸出力量
            double turnOutput = m_pid.calculate(currentYaw);

            // 加上最小靜摩擦力補償 (Feedforward/Deadband)：
            // 如果轉向力量太小 (如 < 0.05)，六輪底盤可能會因為地面摩擦力而動不起來
            if (Math.abs(turnOutput) < 0.05 && !m_pid.atSetpoint()) {
                turnOutput = Math.signum(turnOutput) * 0.05;
            }

            // 呼叫底盤進行原地方向微調 (前進速度 0，僅旋轉)
            m_drive.autoAimDrive(0.0, turnOutput);
        } else {
            // 如果沒看到 AprilTag，就先安全煞停
            m_drive.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        // 返回 false 代表「只要按鈕一直按著，就持續保持對準」
        return false;
    }
}
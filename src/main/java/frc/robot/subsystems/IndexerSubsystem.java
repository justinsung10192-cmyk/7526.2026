package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Falcon Import
import com.ctre.phoenix6.hardware.TalonFX;
// NEO Import
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  // 儲球履帶 (Falcon)
  private final TalonFX m_conveyorMotor = new TalonFX(IndexerConstants.kConveyorCanId);
  // 輸球進入 Shooter (NEO + 1:4 齒輪箱)
  private final SparkMax m_feederMotor = new SparkMax(IndexerConstants.kFeederCanId, MotorType.kBrushless);

  public IndexerSubsystem() {}

  // 履帶控制
  public void setConveyorSpeed(double speed) { m_conveyorMotor.set(speed); }
  // 輸球控制
  public void setFeederSpeed(double speed) { m_feederMotor.set(speed); }

  public void stopAll() {
    m_conveyorMotor.set(0.0);
    m_feederMotor.set(0.0);
  }

  /** 僅開啟履帶 (與 Intake 一起連動，不開啟輸球避免球提早碰到摩擦輪) */
  public Command runConveyorCommand() {
    return this.runEnd(
        () -> setConveyorSpeed(IndexerConstants.kConveyorSpeed),
        () -> setConveyorSpeed(0.0)
    );
  }

  /** 射擊時同時開啟履帶與輸球，把球強勢推入 Shooter */
  public Command feedToShooterCommand() {
    return this.runEnd(
        () -> {
            setConveyorSpeed(IndexerConstants.kConveyorSpeed);
            setFeederSpeed(IndexerConstants.kFeederSpeed);
        },
        this::stopAll
    );
  }
}
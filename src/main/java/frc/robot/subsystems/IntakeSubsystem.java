package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);

  public IntakeSubsystem() {}

  public void setSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stop() {
    m_intakeMotor.set(0.0);
  }

  /** 按住時吸件，放開自動停止 */
  public Command runIntakeCommand() {
    return this.runOnce(
        () -> setSpeed(IntakeConstants.kIntakeSpeed)
    );
  }
  public Command runIntakereCommand() {
    return this.runOnce(
        () -> setSpeed(IntakeConstants.kreIntakeSpeed)
    );
  }


  /** 按住時吐件，放開自動停止 */
  public Command stopIntakeCommand() {
    return this.runOnce(
        () -> stop()
    );
  }
}
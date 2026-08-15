package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// NEO Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

// Falcon Imports
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Intake 滾輪 (Falcon)
  private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.kRollerCanId);
  
  // Intake 收納臂 (NEO + 1:20 齒輪箱)
  private final SparkMax m_pivotMotor = new SparkMax(IntakeConstants.kPivotCanId, MotorType.kBrushless);
  private final RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();
  
  // Pivot PID
  private final ProfiledPIDController m_pivotPID = new ProfiledPIDController(
      IntakeConstants.kPivotKp, IntakeConstants.kPivotKi, IntakeConstants.kPivotKd,
      new TrapezoidProfile.Constraints(180.0, 360.0)
  );
  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(0.0, IntakeConstants.kPivotKg, 0.0);

  private double targetAngle = IntakeConstants.kPivotStowAngle;

  public IntakeSubsystem() {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.encoder.positionConversionFactor(IntakeConstants.kPivotPositionFactor);
    pivotConfig.encoder.velocityConversionFactor(IntakeConstants.kPivotPositionFactor / 60.0);
    
    m_pivotMotor.configure(pivotConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    m_pivotEncoder.setPosition(IntakeConstants.kPivotStowAngle); 
  }

  @Override
  public void periodic() {
    double currentAngle = m_pivotEncoder.getPosition();
    double pidOutput = m_pivotPID.calculate(currentAngle, targetAngle);
    double ffOutput = m_pivotFeedforward.calculate(Math.toRadians(targetAngle), m_pivotPID.getSetpoint().velocity);
    m_pivotMotor.setVoltage(pidOutput + ffOutput);
  }

  // ==== 滾輪控制 ====
  public void setIntakeSpeed(double speed) { m_rollerMotor.set(speed); }
  public void stopIntake() { m_rollerMotor.set(0.0); }

  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /** 複合指令：同時控制放下 Pivot + 啟動 Intake 滾輪 */
  public Command deployAndRunIntakeCommand() {
    return this.runEnd(
        () -> {
            setTargetAngle(IntakeConstants.kPivotDeployAngle);
            setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        },
        () -> {
            stopIntake();
        }
    );
  }

  /** 退件指令 */
  public Command runIntakereCommand() {
    return this.runEnd(() -> setIntakeSpeed(IntakeConstants.kreIntakeSpeed), this::stopIntake);
  }

  /** 單獨收起手臂指令 */
  public Command stowCommand() {
    return this.runOnce(() -> setTargetAngle(IntakeConstants.kPivotStowAngle));
  }
}
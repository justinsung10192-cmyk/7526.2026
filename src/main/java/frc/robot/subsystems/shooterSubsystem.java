package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class shooterSubsystem extends SubsystemBase {
  private final TalonFX m_topMotor = new TalonFX(ShooterConstants.kTopCanId);
  private final TalonFX m_bottomMotor = new TalonFX(ShooterConstants.kBottomCanId);
  
  private final VelocityVoltage m_velocityReq = new VelocityVoltage(0).withSlot(0);
  private final InterpolatingDoubleTreeMap distanceToRpmMap = new InterpolatingDoubleTreeMap();

  public shooterSubsystem() {
    distanceToRpmMap.put(1.5, 2000.0); 
    distanceToRpmMap.put(2.5, 3000.0); 
    distanceToRpmMap.put(4.0, 4200.0); 

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kV = ShooterConstants.kV;

    m_topMotor.getConfigurator().apply(config);
    m_bottomMotor.getConfigurator().apply(config);

    // 💡 使用 Phoenix 6 鏈式呼叫語法，明確指定下馬達反向 (Oppose Master)
    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Current RPM", m_topMotor.getVelocity().getValueAsDouble() * 60.0);
  }

  public void setRPM(double targetRPM) {
    double targetRPS = targetRPM / 60.0;
    m_topMotor.setControl(m_velocityReq.withVelocity(targetRPS));
  }

  public void stop() {
    m_topMotor.set(0.0);
  }

  public Command runShooterCommand(double baseRPM, DoubleSupplier stickInputSupplier) {
    return this.runEnd(
        () -> {
          double trim = -stickInputSupplier.getAsDouble() * ShooterConstants.kTrimSensitivity;
          setRPM(baseRPM + trim);
        },
        this::stop
    );
  }

  public Command autoRangingShootCommand(Vision vision) {
    return this.runEnd(
        () -> {
          double distance = vision.getDistanceToTarget();
          if (distance > 0) {
            double targetRPM = distanceToRpmMap.get(distance);
            setRPM(targetRPM);
            SmartDashboard.putNumber("Shooter/Auto Target RPM", targetRPM);
          } else {
            stop(); 
          }
        },
        this::stop
    );
  }
}
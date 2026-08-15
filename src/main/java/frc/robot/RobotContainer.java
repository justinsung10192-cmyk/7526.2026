package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.TeleopDrive;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final shooterSubsystem m_shooter = new shooterSubsystem();
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final Vision m_vision = new Vision();

  private static final double kFineControlScale = 0.6;//降速率
  public RobotContainer() {
    configureButtonBindings();

    // 預設底盤駕駛指令：
    //WASD控制
    
    m_robotDrive.setDefaultCommand(
        new TeleopDrive(
                m_robotDrive,
                () -> m_driverController.getLeftY(),  // 左搖桿 Y 軸控制前後
                () -> m_driverController.getRightX()  // 右搖桿 X 軸控制旋轉
            )
        );
  }
  

  public DriveSubsystem getDriveSubsystem() {
        return m_robotDrive;
  }

  private void configureButtonBindings() {
    // 按a啟動intake，按b停止intake
    m_driverController.a().whileTrue(m_intake.runIntakeCommand());
    m_driverController.b().whileTrue(m_intake.stopIntakeCommand());
    m_driverController.x().whileTrue(m_intake.runIntakereCommand());
    m_driverController.a().whileTrue(new AutoAim(m_robotDrive, m_vision));

    //Shooter 按鈕控制

    (m_driverController.leftBumper()).whileTrue(
      m_shooter.runShooterCommand(
            ShooterConstants.kStandardSpeed, 
            () -> m_driverController.getRightY()
      )  
    );

    (m_driverController.rightBumper()).whileTrue(
      m_shooter.runShooterCommand(
            ShooterConstants.kFarSpeed, 
            () -> m_driverController.getRightY()
      )  
    );
    
    (m_driverController.y()).whileTrue(
      m_shooter.runShooterreCommand(
            ShooterConstants.kFarSpeed, 
            () -> m_driverController.getRightY()
      )  
    );
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      
      //紅隊洞
    //   m_robotDrive.runDriveCommand(-0.7, 0.0).withTimeout(3.0),
    //   m_robotDrive.runDriveCommand(0.0, -1).withTimeout(0.45),//rot<0右轉,>0左轉
    //   Commands.parallel(
    //     m_intake.runIntakeCommand(),
    //     m_robotDrive.runDriveCommand(-0.5, 0.0)
    //   ).withTimeout(2.5),
    //   m_robotDrive.runDriveCommand(0.5, 0.0).withTimeout(2.5),
    //   m_robotDrive.runDriveCommand(0.0, -1).withTimeout(0.45),
    //   m_robotDrive.runDriveCommand(-0.5, 0.0).withTimeout(3.0),
    //   m_robotDrive.runDriveCommand(0.0, 0.0).withTimeout(2.0),
    //   m_robotDrive.runDriveCommand(0.0, 1).withTimeout(0.45)
    // );

      // 紅隊坡
      // Commands.parallel(
      //   m_intake.runIntakeCommand(),
      //   m_robotDrive.runDriveCommand(-0.5, 0.0)
      // ).withTimeout(5.0),
      // m_robotDrive.runDriveCommand(0.5, 0.0).withTimeout(5.5),
      // m_robotDrive.runDriveCommand(0.0, 1).withTimeout(0.15),
      // m_shooter.runShooterCommand(ShooterConstants.kFarSpeed, () -> 0.0).withTimeout(10.0)

      
      //藍隊洞
    //   m_robotDrive.runDriveCommand(-0.7, 0.0).withTimeout(3.0),
    //   m_robotDrive.runDriveCommand(0.0, 1).withTimeout(0.45),//rot<0右轉,>0左轉
    //   Commands.parallel(
    //     m_intake.runIntakeCommand(),
    //     m_robotDrive.runDriveCommand(-0.5, 0.0)
    //   ).withTimeout(2.5),
    //   m_robotDrive.runDriveCommand(0.5, 0.0).withTimeout(2.5),
    //   m_robotDrive.runDriveCommand(0.0, 1).withTimeout(0.45),
    //   m_robotDrive.runDriveCommand(-0.5, 0.0).withTimeout(3.0),
    //   m_robotDrive.runDriveCommand(0.0, 1).withTimeout(0.45)
    // );

      //藍隊坡
    //   Commands.parallel(
    //     m_intake.runIntakeCommand(),
    //     m_robotDrive.runDriveCommand(-0.5, 0.0)
    //   ).withTimeout(5.0),
    //   m_robotDrive.runDriveCommand(0.5, 0.0).withTimeout(5.5),
    //   m_robotDrive.runDriveCommand(0.0, -1).withTimeout(0.15),
    //   m_shooter.runShooterCommand(ShooterConstants.kFarSpeed, () -> 0.0).withTimeout(10.0)
    // );
    );
  }
}
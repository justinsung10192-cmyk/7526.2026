package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.TeleopDrive;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem(); 
  private final shooterSubsystem m_shooter = new shooterSubsystem();
  private final Vision m_vision = new Vision();
  
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new TeleopDrive(
                m_robotDrive,
                () -> m_driverController.getLeftY(),  
                () -> m_driverController.getRightX()  
            )
        );
  }

  public DriveSubsystem getDriveSubsystem() {
        return m_robotDrive;
  }

  private void configureButtonBindings() {
    // ==========================================
    // 1. 進件 (展開 Pivot + 吸入滾輪 + 履帶向上)
    // ==========================================
    m_driverController.rightTrigger().whileTrue(
        Commands.parallel(
            m_intake.deployCommand(),       
            m_intake.runIntakeCommand(),    
            m_indexer.runConveyorCommand()  // 僅開履帶，不開輸球馬達，避免球提早進 Shooter
        )
    ).onFalse(
        Commands.parallel(
            m_intake.stowCommand()          // 鬆開後自動收起
        )
    );

    m_driverController.x().whileTrue(m_intake.runIntakereCommand()); // 吐件

    // ==========================================
    // 2. 視覺自動測距瞄準
    // ==========================================
    m_driverController.leftTrigger().whileTrue(
        Commands.parallel(
            new AutoAim(m_robotDrive, m_vision),            
            m_shooter.autoRangingShootCommand(m_vision)     
        )
    );

    // ==========================================
    // 3. 射擊推球 (這會強制把 Indexer 履帶跟 Feeder 輸球一起啟動，把球塞進 Shooter)
    // ==========================================
    m_driverController.a().whileTrue(
        m_indexer.feedToShooterCommand() // 當 Shooter 到達轉速時，按下 A 鍵將球射出
    );

    // ==========================================
    // 4. 手動射擊摩擦輪微調
    // ==========================================
    m_driverController.leftBumper().whileTrue(
      m_shooter.runShooterCommand(ShooterConstants.kStandardRPM, () -> m_driverController.getRightY())  
    );

    m_driverController.rightBumper().whileTrue(
      m_shooter.runShooterCommand(4000.0, () -> m_driverController.getRightY()) 
    );
  }

  public Command getAutonomousCommand() {
      return Commands.sequence();
  }
}
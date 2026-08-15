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
import frc.robot.commands.AutoAim;

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

    // 預設底盤手動駕駛 (WASD / 搖桿)
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
    // 1. 進件連動 (按住 Right Trigger)
    // 吸球(deployAndRunIntakeCommand) 與 履帶(runConveyorCommand) 分屬不同 Subsystem，完美併行
    // ==========================================
    m_driverController.rightTrigger().whileTrue(
        Commands.parallel(
            m_intake.deployAndRunIntakeCommand(),    
            m_indexer.runConveyorCommand()
        )
    ).onFalse(
        m_intake.stowCommand() // 鬆開按鈕後收起手臂
    );

    // 吐件 (按住 X 鍵)
    m_driverController.x().whileTrue(m_intake.runIntakereCommand());

    // ==========================================
    // 2. 視覺自動對準與自動測距射擊 (按住 Left Trigger)
    // ==========================================
    m_driverController.leftTrigger().whileTrue(
        Commands.parallel(
            new AutoAim(m_robotDrive, m_vision),            
            m_shooter.autoRangingShootCommand(m_vision)     
        )
    );

    // ==========================================
    // 3. 射擊推球 (按住 A 鍵啟動 Feeder 輸球塞入 Shooter)
    // ==========================================
    m_driverController.a().whileTrue(
        m_indexer.feedToShooterCommand()
    );

    // ==========================================
    // 4. 手動摩擦輪運轉 (Left/Right Bumper)
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
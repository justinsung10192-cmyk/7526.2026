package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_zRotation;

    public TeleopDrive(DriveSubsystem drive, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        m_drive = drive;
        m_xSpeed = xSpeed;
        m_zRotation = zRotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        // 通常搖桿前進是負值 (Y軸拉上為 -1.0)，這裡加負號反轉
        m_drive.arcadeDrive(-m_xSpeed.getAsDouble(), m_zRotation.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
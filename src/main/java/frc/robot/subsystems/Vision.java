package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("photoncamera"); // 請核對相機名稱

    public Vision() {}

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("Vision/Has Target", result.hasTargets());
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Vision/Target ID", result.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("Vision/Target Yaw", result.getBestTarget().getYaw());
        }
    }

    // ========================================================
    // 💡 請確保有加入以下這兩個提供給 AutoAim 呼叫的 Public 方法：
    // ========================================================

    /** 檢查目前相機是否有抓到任何 AprilTag */
    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    /** 取得最佳目標的 Yaw 角度 (偏右為正，偏左為負；沒抓到目標時傳回 0.0) */
    public double getTargetYaw() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }
}
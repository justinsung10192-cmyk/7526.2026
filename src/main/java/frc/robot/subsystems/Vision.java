package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("photoncamera"); 

    public Vision() {}

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        boolean hasTarget = !results.isEmpty() && results.get(results.size() - 1).hasTargets();
        
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget);
        
        if (hasTarget) {
            var bestTarget = results.get(results.size() - 1).getBestTarget();
            SmartDashboard.putNumber("Vision/Target ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Target Yaw", bestTarget.getYaw());
            SmartDashboard.putNumber("Vision/Distance (m)", getDistanceToTarget());
        }
    }

    /** 檢查目前相機是否有抓到任何 AprilTag */
    public boolean hasTarget() {
        var results = camera.getAllUnreadResults();
        return !results.isEmpty() && results.get(results.size() - 1).hasTargets();
    }

    /** 取得最佳目標的 Yaw 角度 (偏右為正，偏左為負；沒抓到目標時傳回 0.0) */
    public double getTargetYaw() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty() && results.get(results.size() - 1).hasTargets()) {
            return results.get(results.size() - 1).getBestTarget().getYaw();
        }
        return 0.0;
    }

    /** 💡 核心方法：根據 Pitch (仰角) 計算機器人到目標的實際距離 (公尺) */
    public double getDistanceToTarget() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty() && results.get(results.size() - 1).hasTargets()) {
            var bestTarget = results.get(results.size() - 1).getBestTarget();
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeightMeters,
                VisionConstants.kTargetHeightMeters,
                VisionConstants.kCameraPitchRadians,
                Math.toRadians(bestTarget.getPitch())
            );
        }
        return -1.0; // 沒看到目標時回傳 -1
    }
}
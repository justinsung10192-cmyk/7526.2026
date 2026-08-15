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
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("Vision/Has Target", result.hasTargets());
        
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Vision/Target ID", result.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("Vision/Target Yaw", result.getBestTarget().getYaw());
            SmartDashboard.putNumber("Vision/Distance (m)", getDistanceToTarget());
        }
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public double getTargetYaw() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double getDistanceToTarget() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeightMeters,
                VisionConstants.kTargetHeightMeters,
                VisionConstants.kCameraPitchRadians,
                Math.toRadians(result.getBestTarget().getPitch())
            );
        }
        return -1.0; 
    }
}
package com.team841.dory.vision;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import com.team841.dory.constants.RC;
import com.team841.dory.lib.limelight.LimelightHelpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelights implements VisionIO {
    NetworkTable charlieTable = NetworkTableInstance.getDefault().getTable(RC.Vision.LimelightFrontName);

    VisionIOInputs inputCache = new VisionIOInputs();

    public VisionIOLimelights() {
        setLLSettings();
    }

    private void setLLSettings() {
        charlieTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Vision.frontPose);

        var gyroAngle = inputCache.gyroAngle;
        var gyroAngularVelocity = inputCache.gyroAngularVelocity;
        try {
            LimelightHelpers.SetRobotOrientation(
                    RC.Vision.LimelightFrontName, gyroAngle.getDegrees(), gyroAngularVelocity, 0, 0, 0, 0);
        } catch (Exception e) {
            return;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.charlieSeesTarget = charlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.charlieSeesTarget) {
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightFrontName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightFrontName);
            if (megatag2 == null || megatag == null) {
                return;
            }
            inputs.charlieMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.charlieMegatagCount = megatag.tagCount;
            inputs.charlieMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.charlieFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }

        this.inputCache = inputs;
        setLLSettings();
    }

    @Override
    public void pollNetworktables() {
        VisionIOInputs inputs = new VisionIOInputs();

        inputs.charlieSeesTarget = charlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.charlieSeesTarget) {
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightFrontName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightFrontName);

            inputs.charlieMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.charlieMegatagCount = megatag.tagCount;
            inputs.charlieMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.charlieFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        setLLSettings();
    }
}

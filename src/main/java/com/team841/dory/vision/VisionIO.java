package com.team841.dory.vision;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean charlieSeesTarget;

        public FiducialObservation[] charlieFiducialObservations;

        public MegatagPoseEstimate charlieMegatagPoseEstimate;

        public int charlieMegatagCount;

        public MegatagPoseEstimate charlieMegatag2PoseEstimates;

        public Rotation2d gyroAngle;
        public double gyroAngularVelocity;
    }

    void updateInputs(VisionIOInputs inputs);

    void pollNetworktables();
}

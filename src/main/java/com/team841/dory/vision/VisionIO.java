package com.team841.dory.vision;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean charlieSeesTarget;
        public boolean gammaSeesTarget;

        public FiducialObservation[] charlieFiducialObservations;
        public FiducialObservation[] gammaFiducialObservations;

        public MegatagPoseEstimate charlieMegatagPoseEstimate;
        public MegatagPoseEstimate gammaMegatagPoseEstimate;

        public int charlieMegatagCount;
        public int gammaMegatagCount;

        public MegatagPoseEstimate charlieMegatag2PoseEstimates;
        public MegatagPoseEstimate gammaMegatag2PoseEstimates;

        public Rotation2d gyroAngle = Rotation2d.kZero;
        public double gyroAngularVelocity;
    }

    void updateInputs(VisionIOInputs inputs);

    //void pollNetworktables();
}

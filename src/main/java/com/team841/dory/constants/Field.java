package com.team841.dory.constants;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units.*;

public class Field {

    public class Positions {
        public static class Reef {
            public static final Pose2d bluePose2d = new Pose2d(Meters.of(4.489337), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d blueTranslation2d = new Translation2d(Meters.of(4.489337), Meters.of(4.025900));
            public static final Pose2d redPose2d = new Pose2d(Meters.of(13.059), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d redTranslation2d = new Translation2d(Meters.of(13.059), Meters.of(4.025900));
        }
    }

    public enum ScoringPositions{
        A(new Pose2d(3.221, 4.193, new Rotation2d(0))),
        B(new Pose2d(3.221, 3.860, new Rotation2d(0))),
        C(new Pose2d(3.702, 3.006, new Rotation2d(Units.degreesToRadians(60)))),
        D(new Pose2d(4.002, 2.842, new Rotation2d(Units.degreesToRadians(60)))),
        E(new Pose2d(4.986, 2.835, new Rotation2d(Units.degreesToRadians(120)))),
        F(new Pose2d(5.263, 3.008, new Rotation2d(Units.degreesToRadians(120)))),
        G(new Pose2d(5.761, 3.853, new Rotation2d(Units.degreesToRadians(180)))),
        H(new Pose2d(5.761, 4.186, new Rotation2d(Units.degreesToRadians(180)))),
        I(new Pose2d(5.259, 5.038, new Rotation2d(Units.degreesToRadians(240)))),
        J(new Pose2d(4.980, 5.211, new Rotation2d(Units.degreesToRadians(240)))),
        K(new Pose2d(3.993, 5.208, new Rotation2d(Units.degreesToRadians(300)))),
        L(new Pose2d(3.716, 5.039, new Rotation2d(Units.degreesToRadians(300))));

        private final Pose2d pose;

        ScoringPositions(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPoseBlue() {
            return pose;
        }

        public Pose2d getPoseRed(){
            return new Pose2d(new Translation2d(Units.feetToMeters(28.7565) + pose.getX(), pose.getY()), pose.getRotation());
        }
    }
}

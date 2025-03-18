package com.team841.dory.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Field {

    public class Positions {
        public static class Reef {
            public static final Pose2d bluePose2d = new Pose2d(Meters.of(4.489337), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d blueTranslation2d = new Translation2d(Meters.of(4.489337), Meters.of(4.025900));
            public static final Pose2d redPose2d = new Pose2d(Meters.of(13.059), Meters.of(4.025900), Rotation2d.kZero);
            public static final Translation2d redTranslation2d = new Translation2d(Meters.of(13.059), Meters.of(4.025900));
        }
    }

    public static class ReefFace {
        char leftFace;
        char rightFace;

        public ReefFace(char leftFace, char rightFace) {
            this.leftFace = leftFace;
            this.rightFace = rightFace;
        }

        public char getLeftFace() {
            return leftFace;
        }

        public char getRightFace() {
            return rightFace;
        }
    }

    public enum ReefLeftRight {
        AB('A', 'B'), CD('C', 'D'), EF('E', 'F'), GH('G', 'H'), IJ('I', 'J'), KL('K', 'L');

        private final ReefFace faceLetters;

        // Constructor
        ReefLeftRight(char left, char right) {
            this.faceLetters = new ReefFace(left, right);
        }

        // Getter method
        public ReefFace getReefFace() {
            return faceLetters;
        }
    }
}

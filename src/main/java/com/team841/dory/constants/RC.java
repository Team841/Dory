package com.team841.dory.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;

public class RC {
    public static final RunType robotType = RunType.DEV;

    public static final CANBus canivoreCANBus = TunerConstants.kCANBus;

    public static Mode getMode() {
        return switch (robotType) {
            case DEV, COMP -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIM -> Mode.SIM;
            case REPLAY -> Mode.REPLAY;
        };
    }
    
    public static final double LOOKBACK_TIME = 1.0;

    public static Supplier<Boolean> isRedAlliance = () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    };

    public class Vision {
        public static final double LOOKBACK_TIME = 1.0;
        public static String LimelightFrontName = "limelight-charlie";
        public static String LimelightBackName = "limelight-gamma";
        // x y z roll pitch yaw
        public static double[] frontPose = {0.0, 0.0, 0.25, 180, 0.0, 0.0};
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RunType {
        SIM, // Simulation
        DEV, // Developer-tuning mode
        COMP, // Comp code, real robot code
        REPLAY
    }
}

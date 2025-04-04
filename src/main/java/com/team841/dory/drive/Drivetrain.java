package com.team841.dory.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team254.vision.VisionFieldPoseEstimate;
import com.team841.dory.constants.Field;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase {
    DriveIO io;

    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    public final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyFieldSpeeds m_choreoFieldCentricSpeeds = new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()// Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    @AutoLogOutput
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    public PIDController controller = new PIDController(4, 0, 0.2);
    public ProfiledPIDController vxController = new ProfiledPIDController(
            10, 0, 0, new TrapezoidProfile.Constraints(
                    4.25, 1.9) // max velocity, max acceleration
    );

    public ProfiledPIDController vyController = new ProfiledPIDController(
            10, 0, 0, new TrapezoidProfile.Constraints(
                    4.25, 1.9) // max velocity, max acceleration
    );

//    public SimpleMotorFeedforward

    public int count = 0;

    PathConstraints constraints = new PathConstraints(
            4.8, 1.8, Units.degreesToRadians(540), Units.degreesToRadians(720));


    private final PIDController xController = new PIDController(6, 0.0, 0.0);
    private final  PIDController yController = new PIDController(6, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5, 0.0, 0.0);

    public Drivetrain(DriveIO io) {
        this.io = io;
        this.controller.setTolerance(0.5);

        this.vxController.setTolerance(Units.inchesToMeters(1));
        this.vyController.setTolerance(Units.inchesToMeters(1));

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
//        this.xController.setTolerance(Units.inchesToMeters(1));
//        this.vyController.setTolerance(Units.inchesToMeters(1));
//        configureAutoBuilder();

        driveHeading.HeadingController.setPID(34.459, 0, 2.5039);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveHeading.HeadingController.setTolerance(0.1, 0.1);
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        telemetry.telemeterize(inputs);
        Logger.processInputs("Drivetrain", inputs);
        Logger.recordOutput("Drive/latencyPeriodicSec", Timer.getTimestamp() - timestamp);

        if (RC.robotType == RC.RunType.DEV) {
//            Logger.recordOutput("Drive/reefAnglePolar", getAngleToReefPolar());
            if (count == 10) {
                Logger.recordOutput("Drive/scoringPose", getPoseToScore(getAngleToReefPolar()));
                count = 0;
            } else {
                count++;
            }

            for (var pose : Field.ScoringPositions.values()) {
                Logger.recordOutput("Pos/" + pose.toString(), pose.getPoseRed());
            }

            Logger.recordOutput("Drive/rotateRate", inputs.Speeds.omegaRadiansPerSecond);
        }

//        if (DriverStation.isDisabled()) {
//            if (!m_hasAppliedOperatorPerspective) {
//                DriverStation.getAlliance().ifPresent(allianceColor -> {
//                    io.setOperatorPerspectiveForward(
//                            allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
//                    m_hasAppliedOperatorPerspective = true;
//                });
//            }
//        }
    }

//    public Command getPathToAutoScore(){
//        return AutoBuilder.pathfindToPose(
//                getPoseToScore(),
//                constraints
//        );
//    }

    public VisionData getVisionData() {
        var visionData = new VisionData();
        visionData.robotPose = inputs.Pose;
        visionData.gyroRotation = inputs.Pose.getRotation();
        visionData.measuredRobotRelativeChassisSpeeds = inputs.Speeds;
        //        visionData.measuredFieldRelativeChassisSpeeds =
        // ChassisSpeeds.fromRobotRelativeSpeeds(visionData.measuredRobotRelativeChassisSpeeds,
        // visionData.gyroRotation);

        visionData.yawRadsPers = inputs.Speeds.omegaRadiansPerSecond;
        return visionData;
    }

    public static class VisionData {
        public Pose2d robotPose;
        public Rotation2d gyroRotation;
        public ChassisSpeeds measuredRobotRelativeChassisSpeeds;
        //        public ChassisSpeeds measuredFieldRelativeChassisSpeeds;
        //        public ChassisSpeeds desiredFieldRelativeChassisSpeeds;
        public double yawRadsPers;
    }

    public double getAngleToReefPolar() {
        boolean isRed = RC.isRedAlliance.get();
        Translation2d robotVector;

        if (isRed) robotVector = inputs.Pose.getTranslation().minus(Field.Positions.Reef.redTranslation2d);
        else robotVector = inputs.Pose.getTranslation().minus(Field.Positions.Reef.blueTranslation2d);

        return Math.atan2(robotVector.getY(), robotVector.getX()) * 57.2957795131;
    }

    public double getAngleToPosePolar(Pose2d targetPose) {
        Translation2d robotVector = inputs.Pose.getTranslation().minus(targetPose.getTranslation());
        return Math.atan2(robotVector.getY(), robotVector.getX());
    }

    public Field.ScoringPositions getScoringPosition(double angle) {
//        double angle = getAngleToReefPolar();
        if (angle >= 0 && angle < 30) {
            return Field.ScoringPositions.H;
        } else if (angle >= 30 && angle < 60) {
            return Field.ScoringPositions.I;
        } else if (angle >= 60 && angle < 90) {
            return Field.ScoringPositions.J;
        } else if (angle >= 90 && angle < 120) {
            return Field.ScoringPositions.K;
        } else if (angle >= 120 && angle < 150) {
            return Field.ScoringPositions.L;
        } else if (angle >= 150 && angle < 180) {
            return Field.ScoringPositions.A;
        } else if (angle >= -180 && angle < -150) {
            return Field.ScoringPositions.B;
        } else if (angle >= -150 && angle < -120) {
            return Field.ScoringPositions.C;
        } else if (angle >= -120 && angle < -90) {
            return Field.ScoringPositions.D;
        } else if (angle >= -90 && angle < -60) {
            return Field.ScoringPositions.E;
        } else if (angle >= -60 && angle < -30) {
            return Field.ScoringPositions.F;
        } else {
            return Field.ScoringPositions.G;
        }
    }

    public boolean getScoringPositionIsRight() {
        Field.ScoringPositions pos = getScoringPosition(this.getAngleToReefPolar());
        switch (pos) {
            case A, C, E, G, I, K -> {
                return false;
            }
            default -> {
                return true;
            }
        }
    }

    public Pose2d getPoseToScore(double angle) {
        if (RC.isRedAlliance.get()) {
            return getScoringPosition(angle).getPoseRed();
        } else {
            return getScoringPosition(angle).getPoseBlue();
        }
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        io.addVisionMeasurement(visionFieldPoseEstimate);
    }

    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> inputs.Pose, // Supplier of current robot pose
                    io::seedFieldRelative, // Consumer for seeding pose against auto
                    () -> inputs.Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> io.setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), new PPHolonomicDriveController(
                                    // PID constants for translation
                                    new PIDConstants(6.75, 0, 0),
                                    // PID constants for rotation
                                    new PIDConstants(5, 0, 0)), config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                (sample.vx + xController.calculate(pose.getX(), sample.x)),
                (sample.vy + yController.calculate(pose.getY(), sample.y)),
                (sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading))
        );

        // Apply the generated speeds
        this.setControl(this.m_choreoFieldCentricSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(sample.moduleForcesX()).withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public ChassisSpeeds getCurrentRobotSpeeds() {
        return inputs.Speeds;
    }

    public Pose2d getPose() {
        return inputs.Pose;
    }

    public void seedFieldCentric() {
        io.seedFieldCentric();
    }

    public Supplier<Pose2d> pose2dSupplier = new Supplier<Pose2d>() {
        @Override
        public Pose2d get() {
            return getPose();
        }
    };

    public ChassisSpeeds getChassisSpeeds() {
        return inputs.Speeds;
    }

    public void setPose(Pose2d pose){
        io.seedFieldRelative(pose);
    }

    public void alignModule(){
        io.pointModulesAtAngle();
    }

    public void setSpeed(ChassisSpeeds speeds){
        this.setControl(this.m_choreoFieldCentricSpeeds.withSpeeds(speeds));
    }

}

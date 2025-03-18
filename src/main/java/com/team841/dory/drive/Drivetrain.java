package com.team841.dory.drive;

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
import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase {
    DriveIO io;

    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    public final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private boolean m_hasAppliedOperatorPerspective = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    public PIDController controller = new PIDController(0.5, 0, 0);

    public Drivetrain(DriveIO io) {
        this.io = io;

        configureAutoBuilder();
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        telemetry.telemeterize(inputs);
        Logger.processInputs("Drivetrain", inputs);
        Logger.recordOutput("Drive/latencyPeriodicSec", Timer.getTimestamp() - timestamp);

        if (DriverStation.isDisabled()) {
            if (!m_hasAppliedOperatorPerspective) {
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    io.setOperatorPerspectiveForward(
                            allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
                    m_hasAppliedOperatorPerspective = true;
                });
            }
        }
    }

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

        return Math.atan2(robotVector.getY(), robotVector.getX());
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        io.addVisionMeasurement(visionFieldPoseEstimate);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> inputs.Pose, // Supplier of current robot pose
                    io::seedFieldRelative, // Consumer for seeding pose against auto
                    () -> inputs.Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> io.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)), config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
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
}

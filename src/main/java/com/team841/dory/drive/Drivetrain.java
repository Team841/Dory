package com.team841.dory.drive;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team254.vision.VisionFieldPoseEstimate;
import com.team841.dory.constants.Field;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.*;

public class Drivetrain extends Drive {

    public Drivetrain(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

        this.vxController.setTolerance(0.5);
        this.vyController.setTolerance(0.5);
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public double Deadband = MaxSpeed * 0.1;

    public double RotationalDeadband = MaxAngularRate * 0.1;

    public Translation2d CenterOfRotation = new Translation2d();

    public boolean DesaturateWheelSpeeds = true;

    public StatusCode runFieldCentric(double toApplyX, double toApplyY, double toApplyOmega) {
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        toApplyX, toApplyY, toApplyOmega,
                        super.getRotation()
                ),
                1.0 / Drive.ODOMETRY_FREQUENCY
        );

        var states = super.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);
        if (DesaturateWheelSpeeds && super.getMaxLinearSpeedMetersPerSec() > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, super.getMaxLinearSpeedMetersPerSec());
        }

        for (int i = 0; i < super.modules.length; ++i) {
            super.modules[i].runSetpoint(states[i]);
        }

        return StatusCode.OK;
    }

    public Rotation2d TargetDirection = new Rotation2d();

    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public StatusCode runFieldCentricFacingAngle(double toApplyX, double toApplyY, Rotation2d angleToFace) {

        double rotationRate = HeadingController.calculate(super.getRotation().getRadians(),
                angleToFace.getRadians(), Timer.getTimestamp());

        double toApplyOmega = rotationRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }

        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        toApplyX, toApplyY, toApplyOmega,
                        super.getRotation()
                ),
                1.0 / Drive.ODOMETRY_FREQUENCY
        );

        var states = super.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);
        if (DesaturateWheelSpeeds && super.getMaxLinearSpeedMetersPerSec() > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, super.getMaxLinearSpeedMetersPerSec());
        }

        for (int i = 0; i < super.modules.length; ++i) {
            super.modules[i].runSetpoint(states[i]);
        }

        return StatusCode.OK;
    }

    public PIDController controller = new PIDController(4, 0, 0.2);
    public ProfiledPIDController vxController = new ProfiledPIDController(
            19.556, 0, 1.9988,
            new TrapezoidProfile.Constraints(
                    3, 2) // max velocity, max acceleration
    );

    public ProfiledPIDController vyController = new ProfiledPIDController(
            19.556, 0, 1.9988,
            new TrapezoidProfile.Constraints(
                    3, 2) // max velocity, max acceleration
    );

    public VisionData getVisionData() {
        var visionData = new VisionData();
        visionData.robotPose = super.getPose();
        visionData.gyroRotation = super.getRotation();
        visionData.measuredRobotRelativeChassisSpeeds = super.getChassisSpeeds();
        //        visionData.measuredFieldRelativeChassisSpeeds =
        // ChassisSpeeds.fromRobotRelativeSpeeds(visionData.measuredRobotRelativeChassisSpeeds,
        // visionData.gyroRotation);

        visionData.yawRadsPers = super.getChassisSpeeds().omegaRadiansPerSecond;
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

        if (isRed) robotVector = super.getPose().getTranslation().minus(Field.Positions.Reef.redTranslation2d);
        else robotVector = super.getPose().getTranslation().minus(Field.Positions.Reef.blueTranslation2d);

        return Math.atan2(robotVector.getY(), robotVector.getX()) * 57.2957795131;
    }

    public double getAngleToPosePolar(Pose2d targetPose){
        Translation2d robotVector = super.getPose().getTranslation().minus(targetPose.getTranslation());
        return Math.atan2(robotVector.getY(), robotVector.getX());
    }

    public Field.ScoringPositions getScoringPosition(double angle){
//        double angle = getAngleToReefPolar();
        if (angle >= 0 && angle < 30){
            return Field.ScoringPositions.H;
        } else if (angle >= 30 && angle < 60){
            return Field.ScoringPositions.I;
        } else if (angle >= 60 && angle < 90){
            return Field.ScoringPositions.J;
        } else if (angle >= 90 && angle < 120){
            return Field.ScoringPositions.K;
        } else if (angle >= 120 && angle < 150){
            return Field.ScoringPositions.L;
        } else if (angle >= 150 && angle < 180){
            return Field.ScoringPositions.A;
        } else if (angle >= -180 && angle < -150) {
            return Field.ScoringPositions.B;
        } else if (angle >= -150 && angle < -120){
            return Field.ScoringPositions.C;
        } else if (angle >= -120 && angle < -90){
            return Field.ScoringPositions.D;
        } else if (angle >= -90 && angle < -60){
            return Field.ScoringPositions.E;
        } else if (angle >= -60 && angle < -30){
            return Field.ScoringPositions.F;
        } else {
            return Field.ScoringPositions.G;
        }
    }

    public Pose2d getPoseToScore(double angle){
        if (RC.isRedAlliance.get()){
            return getScoringPosition(angle).getPoseRed();
        } else {
            return getScoringPosition(angle).getPoseBlue();
        }
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
//        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
//            super.addVisionMeasurement(visionFieldPoseEstimate.getVisionRobotPoseMeters(), visionFieldPoseEstimate.getTimestampSeconds());
//        } else {
            super.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(), visionFieldPoseEstimate.getTimestampSeconds(), visionFieldPoseEstimate.getVisionMeasurementStdDevs());
//        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return super.getChassisSpeeds();
    }

    public Pose2d getPose() {
        return new Pose2d(super.getPose().getTranslation(), super.getRotation());
    }

    public void seedFieldCentric() {
        if (RC.isRedAlliance.get()){
            super.setPose(new Pose2d(0, 0, Rotation2d.k180deg));
        } else {
            super.setPose(Pose2d.kZero);
        }
    }

    public void runVelocity(ChassisSpeeds speeds){
        super.runVelocity(speeds);
    }
}

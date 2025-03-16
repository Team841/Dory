package com.team841.dory.drive;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.vision.VisionFieldPoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveIOReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO{

    AtomicReference<SwerveDriveState> telemetryCache_ = new AtomicReference<>();

    public DriveIOReal(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        
        this.registerTelemetry(telemetryConsumer_); 
    }

    Consumer<SwerveDriveState> telemetryConsumer_ = swerveDriveState -> {
        telemetryCache_.set(swerveDriveState.clone());
    };

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.fromSwerveDriveState(telemetryCache_.get());
        inputs.gyroAngle = inputs.Pose.getRotation().getDegrees();
    }

    @Override
    public void logModules(SwerveDriveState state) {
        final String[] moduleNames = { "Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR" };
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput(moduleNames[i] + " Absolute Encoder Angle", getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(moduleNames[i] + " Steering Angle", state.ModuleStates[i].angle);
            Logger.recordOutput(moduleNames[i] + " Target Steering Angle", state.ModuleTargets[i].angle);
            Logger.recordOutput(moduleNames[i] + " Drive Velocity", state.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(moduleNames[i] + " Target Drive Velocity", state.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    @Override
    public void seedFieldCentric() {
        super.seedFieldCentric();
    }

    /**
     * Sets the odometry pose
     */
    @Override
    public void seedFieldRelative(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

   

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return new RunCommand(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }

    @Override
    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            addVisionMeasurement(visionFieldPoseEstimate.getVisionRobotPoseMeters(), visionFieldPoseEstimate.getTimestampSeconds());
        } else {
            addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    visionFieldPoseEstimate.getTimestampSeconds(),
                    visionFieldPoseEstimate.getVisionMeasurementStdDevs());
        }
    }
    
    
}

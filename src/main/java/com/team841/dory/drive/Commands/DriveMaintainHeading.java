package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Drive command that maintains the robot facing angle when there is no turn control during teleop mode.
 * This forces the drivetrain to keep its angle and prevents it from slowly drifting.
 */
public class DriveMaintainHeading extends Command {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    Drivetrain drivetrain;
    DoubleSupplier controlXSupplier, controlYSupplier, controlAngularVelocitySupplier;
    Optional<Rotation2d> mHeadingSetpoint;
    Double controlX, controlY, controlAngularVelocity;

    BooleanSupplier normalDrive;

    public DriveMaintainHeading(
            Drivetrain drivetrain,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier angularVelocity,
            BooleanSupplier normalDrive) {
        this.drivetrain = drivetrain;
        this.controlXSupplier = velocityX;
        this.controlYSupplier = velocityY;
        this.controlAngularVelocitySupplier = angularVelocity;
        this.normalDrive = normalDrive;

        driveHeading.HeadingController.setPID(34.459, 0, 2.5039);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain);
        setName("DriveMaintainHeading");
    }

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    private final SwerveRequest.FieldCentricFacingAngle driveHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {

        // Controller value output is between -1 to 1, we need to scale to max speed
        // The current speed of the robot in teleop is directly proportional to the joystick input.
        this.controlX = this.controlXSupplier.getAsDouble();
        this.controlY = this.controlYSupplier.getAsDouble();
        this.controlAngularVelocity = this.controlAngularVelocitySupplier.getAsDouble();
        double velocityX = controlX * MaxSpeed;
        double velocityY = controlY * MaxSpeed;
        double angularVelocity = controlAngularVelocity * MaxAngularRate;

        // Switch the sign of the velocity depending on which alliance color you are because forward is away from you
        // always
        double throttleFieldFrame = RC.isRedAlliance.get() ? -velocityX : velocityX;
        double strafeFieldFrame = RC.isRedAlliance.get() ? -velocityY : velocityY;
//        if (!normalDrive.getAsBoolean()) {
//            if (Math.abs(controlAngularVelocity) > this.kSteerJoystickDeadband) {
//                mJoystickLastTouched = Timer.getFPGATimestamp();
//            }
//            if (Math.abs(controlAngularVelocity) > kSteerJoystickDeadband || (epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25) && Math.abs(drivetrain.getCurrentRobotSpeeds().omegaRadiansPerSecond) > Math.toRadians(10))) {
//                drivetrain.setControl(drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withRotationalRate(angularVelocity));
//                mHeadingSetpoint = Optional.empty();
//                Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
//            } else {
//                if (mHeadingSetpoint.isEmpty()) {
//                    mHeadingSetpoint = Optional.of(drivetrain.getPose().getRotation().rotateBy(Rotation2d.k180deg));
//                }
//                drivetrain.setControl(driveHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withTargetDirection(mHeadingSetpoint.get()));
//                Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
//                Logger.recordOutput(
//                        "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
//            }
//        } else {
        drivetrain.setControl(drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withRotationalRate(angularVelocity));
//        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}

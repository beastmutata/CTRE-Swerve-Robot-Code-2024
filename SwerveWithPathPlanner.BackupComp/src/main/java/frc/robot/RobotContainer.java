// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandJoystick joystick = new CommandJoystick(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(); //Climber Subsystem

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.button(1).whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.button(11).whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))));

    // reset the field-centric heading on left bumper press
    joystick.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.button(15).and(joystick.button(13)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.button(15).and(joystick.button(14)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.button(14).and(joystick.button(13)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.button(14).and(joystick.button(15)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick.button(6).onTrue(new InstantCommand(() -> climberSubsystem.setLeftSpeed(0.5), climberSubsystem));
    joystick.button(6).onTrue(new InstantCommand(() -> climberSubsystem.setRightSpeed(0.5), climberSubsystem));

    joystick.button(9).onTrue(new InstantCommand(() -> climberSubsystem.setLeftSpeed(-0.5), climberSubsystem));
    joystick.button(9).onTrue(new InstantCommand(() -> climberSubsystem.setRightSpeed(-0.5), climberSubsystem));

  joystick.button(10).onTrue(new InstantCommand(() -> climberSubsystem.setLeftSpeed(0.0), climberSubsystem));
  joystick.button(10).onTrue(new InstantCommand(() -> climberSubsystem.setRightSpeed(0.0), climberSubsystem));


  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}

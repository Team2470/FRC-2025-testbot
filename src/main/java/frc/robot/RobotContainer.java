// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ForwardReference;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Aligntoreef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ReduxColorSensor;
import frc.robot.subsystems.CoralThing;
import frc.robot.subsystems.CoralThing_FXS;
//import frc.robot.subsystems.TalonFXSTest;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public double target = 40;

  // private final Elevator elevator1 = new Elevator(1, true);
  // private final Elevator elevator2 = new Elevator(2, false);  


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final ReduxColorSensor colorSensor = new ReduxColorSensor();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    //private final CoralThing_FXS coralthing_Fxs = new CoralThing_FXS(0, 0, false);

    //private final TalonFXSTest talonFXS = new TalonFXSTest(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    public RobotContainer() {
        // addValuesToDashboard();
        configureBindings();
    }


    private void configureBindings() {
    //
    // Drive Train
    //

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    BooleanSupplier slowModeSupplier = () -> joystick.getHID().getXButton();

    DoubleSupplier rotationSupplier = () -> {
        double leftTrigger = joystick.getHID().getLeftTriggerAxis();
        double rightTrigger = joystick.getHID().getRightTriggerAxis();

        double rotate = 0.0;
        if (leftTrigger < rightTrigger) {
          rotate = -rightTrigger;
        } else {
          rotate = leftTrigger;
        }

        return rotateFilter.calculate(rotate) * MaxAngularRate;
    };

    Supplier<Translation2d> translationSupplier = () -> {
        // Read gamepad joystick state, and apply slew rate limiters
        Translation2d move = new Translation2d(
          // X Move Velocity - Forward
          MathUtil.applyDeadband(xFilter.calculate(-joystick.getHID().getLeftY()),.05),
          // Y Move Velocity - Strafe
          MathUtil.applyDeadband(yFilter.calculate(-joystick.getHID().getLeftX()),.05)
        );
        
        return new Translation2d(
            // Scale the speed of the robot by using a quadratic input curve.
            // and convert the joystick values -1.0-1.0 to Meters Per Second
            Math.pow(move.getNorm(), 2) * MaxSpeed, 
            // Get the direction the joystick is pointing 
            move.getAngle()
        );
    };

    // Field-centric by default
    final var fieldCentric = new SwerveRequest.FieldCentric();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(()->{
            var translation = translationSupplier.get();


            double xMove = translation.getX();
            double yMove = translation.getY();
            double rotate = rotationSupplier.getAsDouble();

            if(slowModeSupplier.getAsBoolean()) {
              xMove *= 0.5;
              yMove *= 0.5;
              rotate *= 0.25;
            } else{
              xMove *= 1.0;
              yMove *= 1.0;
              rotate *= 0.5;
            }

            return fieldCentric
              .withVelocityX(xMove)
              .withVelocityY(yMove)
              .withRotationalRate(rotate);
        })
    );

    // Robot Centric when pressing A
    final var robotCentric = new SwerveRequest.RobotCentric();
    //joystick.x().onTrue(coralthing_Fxs.slowMotorAtSensorCommand());
    //joystick.x().onTrue(coralthing_Fxs.slowMotorAtSensorCommand(joystick.y().getAsBoolean()));
    //joystick.y().onTrue(coralthing_Fxs.stopMotorCommand());
    //joystick.y().whileTrue(coralthing.ignoreLimitCommand());
    // joystick.leftBumper().whileTrue(elevatorUpCommand());
    // joystick.rightBumper().whileTrue(elevatorDownCommand());
    // joystick.leftBumper().whileTrue(elevatorToPostitonCommandDash(75));
    // joystick.rightBumper().whileTrue(elevatorToPostitonCommandDash(35));
    // joystick.y().whileTrue(elevatorToPostitonCommandDash());

    joystick.a().whileTrue(drivetrain.applyRequest(()->{
        var translation = translationSupplier.get();

        double xMove = translation.getX();
        double yMove = translation.getY();
        double rotate = rotationSupplier.getAsDouble();

        if(slowModeSupplier.getAsBoolean()) {
          xMove *= 0.5;
          yMove *= 0.5;
          rotate *= 0.25;
        } else{
          xMove *= 1.0;
          yMove *= 1.0;
          rotate *= 0.5;
        }

        return robotCentric
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withRotationalRate(rotate);
    }));

    // // Align to Amp by pressing left bumper
    // final var alignToAmp = new SwerveRequest.FieldCentricFacingAngle();
    // alignToAmp.HeadingController.setPID(0,0,0);
    // alignToAmp.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    // // This makes angles specified below to be in referece to the driver. This means
    // // angles will get automatically flipped if we are Red or Blue alliance.
    // // If set to ForwardReference.RedAlliance then the angles will not be flipped.
    // // TODO what is zero degrees?

    // // alignToAmp.ForwardReference = ForwardReference.RedAlliance; 

    // joystick.leftBumper().whileTrue(drivetrain.applyRequest(()->{
    //     var translation = translationSupplier.get();

    //     double xMove = translation.getX();
    //     double yMove = translation.getY();

    //     if(slowModeSupplier.getAsBoolean()) {
    //       xMove *= 0.5;
    //       yMove *= 0.5;
    //     } else{
    //       xMove *= 1.0;
    //       yMove *= 1.0;
    //     }

    //     return alignToAmp
    //       .withVelocityX(xMove)
    //       .withVelocityY(yMove)
    //       .withTargetDirection(Rotation2d.fromDegrees(90)
    //     );
    // }));

    // Align to Source by pressing right bumper
    final var alignToSource = new SwerveRequest.FieldCentricFacingAngle();
    // This makes angles specified below to be in referece to the driver. This means
    // angles will get automatically flipped if we are Red or Blue alliance.
    // If set to ForwardReference.RedAlliance then the angles will not be flipped.
    // TODO what is zero degrees?

    // alignToAmp.ForwardReference = ForwardReference.OperatorPerspective; 

    // joystick.rightBumper().whileTrue(drivetrain.applyRequest(()->{
    //     var translation = translationSupplier.get();

    //     double xMove = translation.getX();
    //     double yMove = translation.getY();

    //     if(slowModeSupplier.getAsBoolean()) {
    //       xMove *= 0.5;
    //       yMove *= 0.5;
    //     } else{
    //       xMove *= 1.0;
    //       yMove *= 1.0;
    //     }

    //     // The angle we need to face needs to change depending on which alliance we are
    //     // on.
    //     double angle = -120.0;
    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //       angle = -60.0;
    //     }

    //     return alignToSource.withDeadband(0.05)
    //       .withVelocityX(xMove)
    //       .withVelocityY(yMove)
    //       .withTargetDirection(Rotation2d.fromDegrees(angle)
    //     );
    // }));

    // X-Stop
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // Reset the field-centric heading on start press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    joystick.y().whileTrue(new Aligntoreef(drivetrain));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    
    // public Command elevatorUpCommand(){
    //   return new ParallelCommandGroup(
    //     elevator1.openLoopCommand(2),
    //     elevator2.openLoopCommand(2)
    //   );
    // }

    // public Command elevatorDownCommand(){
    //   return new ParallelCommandGroup(
    //     elevator1.openLoopCommand(-0.5),
    //     elevator2.openLoopCommand(-0.5)
    //   );
    // }
    // public Command elevatorToPostitonCommand(){
    //   return new ParallelCommandGroup(
    //     elevator1.pidCommand(50),
    //     elevator2.pidCommand(50)
    //   );
    // }
    // public Command elevatorToPostitonCommandDash(double target){
    //   return new ParallelCommandGroup(
    //     elevator1.pidCommand(target),
    //     elevator2.pidCommand(target)
    //   );
    // }

    public void robotPeriodic() {
      SmartDashboard.putNumber("target", target);
      SmartDashboard.putBoolean("y button press", joystick.y().getAsBoolean());
    }
}


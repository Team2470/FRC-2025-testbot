// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.pheonix6.swerve.ModifiedRobotCentricFacingAngle;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
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
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
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

    private final CommandXboxController controller = new CommandXboxController(0);
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

    BooleanSupplier slowModeSupplier = () -> controller.getHID().getXButton();

    DoubleSupplier rotationSupplier = () -> {
      double leftTrigger = controller.getHID().getLeftTriggerAxis();
      double rightTrigger = controller.getHID().getRightTriggerAxis();

      SmartDashboard.putNumber("Left Trigger", leftTrigger);
      SmartDashboard.putNumber("Right Trigger", rightTrigger);

      double rotate = 0.0;
      if (leftTrigger < rightTrigger) {
        rotate = -rightTrigger;
      } else {
        rotate = leftTrigger;
      }

      rotate = Math.copySign(rotate * rotate, rotate);
      rotate = rotateFilter.calculate(rotate) * MaxAngularRate;
      SmartDashboard.putNumber("Rotate", rotate);
      return rotate;
    };

    Supplier<Optional<Translation2d>> translationSupplier = () -> {
      // Read gamepad joystick state, and apply slew rate limiters

      // X Move Velocity - Forward
      double xMove = xFilter.calculate(-controller.getHID().getLeftY());

      // Y Move Velocity - Strafe
      double yMove = yFilter.calculate(-controller.getHID().getLeftX());

      // if (controller.getHID().getLeftBumperButton()) {
      //   yMove = 0;
      // }

      if (xMove == 0 && yMove == 0) {
        return Optional.empty();
      }

      Translation2d move = new Translation2d(xMove, yMove);

      return Optional.of(new Translation2d(
          // Scale the speed of the robot by using a quadratic input curve.
          // and convert the joystick values -1.0-1.0 to Meters Per Second
          Math.pow(move.getNorm(), 2) * MaxSpeed,
          // Get the direction the joystick is pointing
          move.getAngle()));
    };

    // Field-centric by default
    final var fieldCentric = new SwerveRequest.FieldCentric()
        // .withDeadband(MaxSpeed * 0.1)
        // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    Rotation2d latchedHeading;
    final var fieldCentricHeadingStable = new SwerveRequest.FieldCentricFacingAngle()
        // .withDeadband(MaxSpeed * 0.1)
        // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withHeadingPID(10, 0, 0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);


    final var fieldCentricIdle = new SwerveRequest.Idle();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          if (DriverStation.isAutonomousEnabled()) {
            return fieldCentricIdle;
          }

          var translation = translationSupplier.get();

          double rotate = rotationSupplier.getAsDouble();
          double xMove = 0;
          double yMove = 0;

          if (translation.isPresent()) {
            xMove = translation.get().getX();
            yMove = translation.get().getY();
          }

          if (Math.abs(rotate) < MaxAngularRate * 0.05 && Math.abs(xMove) < MaxSpeed * 0.05 && Math.abs(yMove) < MaxSpeed * 0.05) {
            return fieldCentricIdle;
          }


          if (slowModeSupplier.getAsBoolean()) {
            xMove *= 0.5;
            yMove *= 0.5;
            rotate *= 0.25;
          } else {
            xMove *= 1.0;
            yMove *= 1.0;
            rotate *= 0.5;
          }

          // if (Math.abs(rotate) < MaxAngularRate * 0.05) {

          //   if (latchedHeading == null) {
          //     latchedHeading = Rotation2d.fromDegrees(yMove)
          //   }

          //   return fieldCentricHeadingStable
          //   .withVelocityX(xMove)
          //   .withVelocityY(yMove)
          //   .withTargetDirection(Rotatio)
          // } else {
          //   latchedHeading = null;
          // }

          return fieldCentric
              .withVelocityX(xMove)
              .withVelocityY(yMove)
              .withRotationalRate(rotate);
        }).withName("Field Centric with GamePad"));



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

    controller.a().whileTrue(drivetrain.applyRequest(() -> {
      var translation = translationSupplier.get();

      double rotate = rotationSupplier.getAsDouble();
      double xMove = 0;
      double yMove = 0;

      if (translation.isPresent()) {
        xMove = translation.get().getX();
        yMove = translation.get().getY();
      }

      if (slowModeSupplier.getAsBoolean()) {
        xMove *= 0.5;
        yMove *= 0.5;
        rotate *= 0.25;
      } else {
        xMove *= 1.0;
        yMove *= 1.0;
        rotate *= 0.5;
      }

      return robotCentric
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withRotationalRate(rotate);
    }).withName("Robot Centric with GamePad"));


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
    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // Reset the field-centric heading on start press
    controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    controller.y().whileTrue(new Aligntoreef(drivetrain));
    
    final ModifiedRobotCentricFacingAngle swerveAlign = new ModifiedRobotCentricFacingAngle()
      .withHeadingPID(10, 0, 0)
      .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    controller.leftBumper().whileTrue(drivetrain.applyRequest(()-> {
        return swerveAlign
          .withVelocityX(0)
          .withVelocityY(0)
          .withTargetDirection(Rotation2d.fromDegrees(45));
    }));

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
      SmartDashboard.putBoolean("y button press", controller.y().getAsBoolean());
    }
}


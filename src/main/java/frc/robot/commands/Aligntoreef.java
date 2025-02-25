// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aligntoreef extends SequentialCommandGroup {
  private final static String kLimelight = "limelight-shooter";
  private final PIDController m_txPID = new PIDController(0.1, 0, 0);
  private final PIDController m_tyPID = new PIDController(0.1,0,0);


  final SwerveRequest.RobotCentricFacingAngle swerveAlign = new SwerveRequest.RobotCentricFacingAngle()
    .withHeadingPID(5, 0, 0)
    .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Creates a new Aligntoreef. */
  public Aligntoreef(CommandSwerveDrivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    setName("Aligntoreef");
    addCommands(
      new WaitUntilCommand(()-> LimelightHelpers.getTV(kLimelight)),
      Commands.runOnce(() -> {
        m_txPID.reset();
        m_txPID.setTolerance(1);

        m_tyPID.reset();
        m_tyPID.setTolerance(1);
      }),
      drive.applyRequest(() -> {
        double xMove = MathUtil.clamp(
          m_tyPID.calculate(LimelightHelpers.getTY(kLimelight), 0), -1,1
        );
        double yMove = MathUtil.clamp(
          m_txPID.calculate(LimelightHelpers.getTX(kLimelight), 0), -1,1
        );
        
        SmartDashboard.putNumber("AlignToReef tx error", m_txPID.getPositionError());
        SmartDashboard.putNumber("AlignToReef ty error", m_tyPID.getPositionError());
        SmartDashboard.putNumber("AlignToReef xMove", xMove);
        SmartDashboard.putNumber("AlignToReef yMove", yMove);
        
        return swerveAlign
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withTargetDirection(Rotation2d.fromDegrees(0));
      })
    );
  }
}

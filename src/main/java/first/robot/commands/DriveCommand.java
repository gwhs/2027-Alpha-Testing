// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import first.robot.swerve.SwerveSubsystem;
import first.robot.swerve.SwerveSubsystem.RotationTarget;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final CommandXboxController controller;

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private final double maxSpeed = 4.5; // m/s
  private final double maxAngularSpeed = 2.5 * Math.PI; // radians / sec

  private final double deadband = 0.06;

  private final PIDController robotHeadingController = new PIDController(0.2, 0, 0);

  private final SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(1);

  private boolean shouldResetRateLimiter = true;

  /** Creates a new DriveCommand. */
  public DriveCommand(SwerveSubsystem drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    robotHeadingController.enableContinuousInput(-180, 180);

    // if(drivetrain.isLimitAcceleration()){}

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = -controller.getLeftY();
    double yInput = -controller.getLeftX();
    double rotationInput = -controller.getRightX();

    xInput = MathUtil.applyDeadband(xInput, deadband);
    yInput = MathUtil.applyDeadband(yInput, deadband);
    rotationInput = MathUtil.applyDeadband(rotationInput, deadband);

    // Auto Rotation
    boolean hasRotationInput = Math.abs(controller.getRightX()) > 0.1;

    if (drivetrain.getRotationTarget() != RotationTarget.NORMAL && !hasRotationInput) {
      double currentRobotHeading = drivetrain.getState().Pose.getRotation().getDegrees();

      robotHeadingController.setSetpoint(drivetrain.getGoalHeading());

      double pidOutput = robotHeadingController.calculate(currentRobotHeading);
      rotationInput = pidOutput;
    }

    if (drivetrain.isSlowMode()) {
      xInput = xInput * drivetrain.getTranslationSlowFactor();
      yInput = yInput * drivetrain.getTranslationSlowFactor();
      rotationInput = rotationInput * drivetrain.getRotationalSlowFactor();
    }

    // Limit Acceleration
    if (drivetrain.isLimitAcceleration()) {
      if (shouldResetRateLimiter) {
        shouldResetRateLimiter = false;
        xVelocityLimiter.reset(xInput);
        yVelocityLimiter.reset(yInput);
        angularVelocityLimiter.reset((rotationInput));
      }
      xInput = xVelocityLimiter.calculate(xInput);
      yInput = xVelocityLimiter.calculate(yInput);
      rotationInput = angularVelocityLimiter.calculate(rotationInput);
    } else {
      shouldResetRateLimiter = true;
    }

    double xVelocity = xInput * maxSpeed;
    double yVelocity = yInput * maxSpeed;
    double rotationVelocity = rotationInput * maxAngularSpeed;

    drivetrain.setControl(
        fieldCentric
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

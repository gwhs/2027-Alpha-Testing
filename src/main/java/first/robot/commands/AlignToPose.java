package first.robot.commands;

import static org.wpilib.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import first.robot.swerve.SwerveSubsystem;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.MathUtil;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignToPose extends Command {

  private final Supplier<Pose2d> targetPose;
  private final double ELEVATOR_UP_SLEW_RATE = 1;

  private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final DoubleSupplier elevatorHeight;

  private boolean resetLimiter = true;
  private CommandXboxController driverController;

  private SwerveSubsystem drivetrain;
  public Constraints constraints = new TrapezoidProfile.Constraints(3, 2);
  public ProfiledPIDController PID_X = new ProfiledPIDController(3.0, 0, 0, constraints);
  public ProfiledPIDController PID_Y = new ProfiledPIDController(3.0, 0, 0, constraints);

  public PIDController PID_Rotation = new PIDController(0.05, 0, 0);
  private double maxSpeed = MetersPerSecond.of(4.73).in(MetersPerSecond);

  private double maxAngularRate = 1.0 * Math.PI;

  private long startTime;

  public static final double PID_MAX = 0.44;
  public static final double PID_ROTATION_MAX = 0.70;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.05)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(
      Supplier<Pose2d> poseSupplier,
      SwerveSubsystem drivetrain,
      DoubleSupplier elevatorHeight,
      CommandXboxController driverController) {

    this.drivetrain = drivetrain;
    this.targetPose = poseSupplier;
    this.driverController = driverController;
    this.elevatorHeight = elevatorHeight;

    PID_Rotation.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);
  }

  /**
   * @return if it is at pose true if not false
   */
  public boolean isAtTargetPose() {
    boolean isAtX = PID_X.atSetpoint();
    boolean isAtY = PID_Y.atSetpoint();
    boolean isAtRotation = PID_Rotation.atSetpoint();
    DogLog.log("Align/atX", isAtX);
    DogLog.log("Align/atY", isAtY);
    DogLog.log("Align/atRotation", isAtRotation);

    if (isAtX && isAtY && isAtRotation) {
      return true;
    }
    return false;
  }

  public boolean isJoystickActive() {
    long now = System.currentTimeMillis();
    if (now - startTime < 1000) { // if under 1 second, then joystick shouldn't be considered active
      return false;
    }
    double xVelocity = driverController.getLeftY();
    double yVelocity = driverController.getLeftX();

    if (Math.abs(yVelocity) > 0.1 || Math.abs(xVelocity) > 0.1) {
      return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    Pose2d tp = targetPose.get();
    ChassisVelocities currentVelocity =
      drivetrain.getState().Velocity.toFieldRelative(drivetrain.getState().Pose.getRotation());


    double predicted_X =
        (tp.getX() - drivetrain.getState().Pose.getX()) * 0.3
            + drivetrain.getState().Pose.getX();
    double predicted_Y =
        (tp.getY() - drivetrain.getState().Pose.getY()) * 0.3
            + drivetrain.getState().Pose.getY();

    PID_X.reset(predicted_X, currentVelocity.vx * 0.4);
    PID_Y.reset(predicted_Y, currentVelocity.vy * 0.4);
    PID_X.setGoal(tp.getX());
    PID_Y.setGoal(tp.getY());
    PID_Rotation.setSetpoint(tp.getRotation().getDegrees());
    DogLog.log("Align/Target Pose", targetPose.get());
  }

  @Override
  public void execute() {
    Pose2d currPose;
    currPose = drivetrain.getState().Pose;
    double currX = currPose.getX();
    double currY = currPose.getY();
    Double currRotation = currPose.getRotation().getDegrees();

    double PIDXOutput = Math.clamp(PID_X.calculate(currX), -PID_MAX, PID_MAX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = Math.clamp(PID_Y.calculate(currY), -PID_MAX, PID_MAX);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        Math.clamp(PID_Rotation.calculate(currRotation), -PID_ROTATION_MAX, PID_ROTATION_MAX);
    double angularVelocity = PIDRotationOutput;
    DogLog.log("Align/PIDRotationoutput", PIDRotationOutput);

    if (elevatorHeight.getAsDouble() > 0.4) {
      if (resetLimiter) {
        resetLimiter = false;
        xVelocityLimiter.reset(xVelocity);
        yVelocityLimiter.reset(yVelocity);
        angularVelocityLimiter.reset(angularVelocity);
      }
      xVelocity = Math.clamp(xVelocity, -0.2, 0.2);
      yVelocity = Math.clamp(yVelocity, -0.2, 0.2);
      angularVelocity = Math.clamp(angularVelocity, -0.4, 0.4);

      xVelocity = xVelocityLimiter.calculate(xVelocity);
      yVelocity = yVelocityLimiter.calculate(yVelocity);
      angularVelocity = angularVelocityLimiter.calculate(angularVelocity);
    } else {
      resetLimiter = true;
    }

    Optional<Alliance> ally = MatchState.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.RED) {
            xVelocity = -xVelocity * maxSpeed;
            yVelocity = -yVelocity * maxSpeed;
            angularVelocity = angularVelocity * maxAngularRate;
        }
        if (ally.get() == Alliance.BLUE) {
            xVelocity = -xVelocity * maxSpeed;
            yVelocity = -yVelocity * maxSpeed;
            angularVelocity = angularVelocity * maxAngularRate;
        }
    } else {
        // No alliance color available yet
    }

    angularVelocity = Math.clamp(angularVelocity, -maxAngularRate, maxAngularRate);
    DogLog.log("Align/xVelocity", xVelocity);
    DogLog.log("Align/yVelocity", yVelocity);
    DogLog.log("Align/angularVelocity", angularVelocity);
    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(angularVelocity)); // Drive counterclockwise with negative X (left)
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0.00).withVelocityY(0.00).withRotationalRate(0.00));
  }

  @Override
  public boolean isFinished() {
    if (isJoystickActive()) {
      return false;
    }
    return false;
  }
}

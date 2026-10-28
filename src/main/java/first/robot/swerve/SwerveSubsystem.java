package first.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.alerts.AlertableCollection;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Current;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {

  public enum RotationTarget {
    NORMAL,
    FORTY_FIVE,
    RED_HUB
  }

  private RotationTarget rotationTarget = RotationTarget.NORMAL;

  private final AlertableCollection deviceAlerts = new AlertableCollection("Swerve");

  private TalonFX frontLeftDrive = this.getModule(0).getDriveMotor();
  private TalonFX frontLeftTurn = this.getModule(0).getSteerMotor();
  private CANcoder frontLeftEncoder = this.getModule(0).getEncoder();
  private TalonFX frontRightDrive = this.getModule(1).getDriveMotor();
  private TalonFX frontRightTurn = this.getModule(1).getSteerMotor();
  private CANcoder frontRightEncoder = this.getModule(1).getEncoder();
  private TalonFX backLeftDrive = this.getModule(2).getDriveMotor();
  private TalonFX backLeftTurn = this.getModule(2).getSteerMotor();
  private CANcoder backLeftEncoder = this.getModule(2).getEncoder();
  private TalonFX backRightDrive = this.getModule(3).getDriveMotor();
  private TalonFX backRightTurn = this.getModule(3).getSteerMotor();
  private CANcoder backRightEncoder = this.getModule(3).getEncoder();
  private Pigeon2 pigeon = this.getPigeon2();

  private final StatusSignal<Current> frontLeftDriveStatorCurrent = frontLeftDrive.getStatorCurrent();
  private final StatusSignal<Current> frontLeftTurnStatorCurrent = frontLeftTurn.getStatorCurrent();
  private final StatusSignal<Current> frontRightDriveStatorCurrent = frontRightDrive.getStatorCurrent();
  private final StatusSignal<Current> frontRightTurnStatorCurrent = frontRightTurn.getStatorCurrent();
  private final StatusSignal<Current> backLeftDriveStatorCurrent = backLeftDrive.getStatorCurrent();
  private final StatusSignal<Current> backLeftTurnStatorCurrent = backLeftTurn.getStatorCurrent();
  private final StatusSignal<Current> backRightDriveStatorCurrent = backRightDrive.getStatorCurrent();
  private final StatusSignal<Current> backRightTurnStatorCurrent = backRightDrive.getStatorCurrent();
  
  private boolean isLimitAcceleration = false;

  private double translationSlowFactor = 1;
  private double rotationSlowFactor = 1;
  private boolean isSlowMode = false;

  private final double defaultSlowFactor = 0.25;

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.ZERO;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotVelocity m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);

    registerAlerts();

    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();

    BaseStatusSignal.setUpdateFrequencyForAll(50, 
      frontLeftDriveStatorCurrent, 
      frontLeftTurnStatorCurrent, 
      frontRightDriveStatorCurrent, 
      frontRightTurnStatorCurrent, 
      backLeftDriveStatorCurrent, 
      backLeftTurnStatorCurrent,
      backRightDriveStatorCurrent,
      backRightTurnStatorCurrent);
  }

  private void configureAutoBuilder() {
    var config =
        new RobotConfig(
            75,
            6.8,
            new ModuleConfig(
                0.0508, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
            getModuleLocations());

    AutoBuilder.configure(
        () -> getState().Pose, // Supplier of current robot pose
        this::resetPose, // Consumer for seeding pose against auto
        () -> getState().Velocity, // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) ->
            setControl(
                m_pathApplyRobotSpeeds
                    .withVelocity(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(10, 0, 0),
            // PID constants for rotation
            new PIDConstants(7, 0, 0)),
        config,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
        this // Subsystem for requirements
        );
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  private void registerAlerts() {
    /* register alerts for all the devices in the drivetrain */
    for (final var module : getModules()) {
      deviceAlerts
          .withAlertable(module.getDriveMotor())
          .withAlertable(module.getSteerMotor())
          .withAlertable(module.getEncoder());
    }
    deviceAlerts.withAlertable(getPigeon2());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      frontLeftDriveStatorCurrent,
      frontLeftTurnStatorCurrent,
      frontRightDriveStatorCurrent, 
      frontRightTurnStatorCurrent, 
      backLeftDriveStatorCurrent, 
      backLeftTurnStatorCurrent,
      backRightDriveStatorCurrent,
      backRightTurnStatorCurrent);

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {
      MatchState.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorForwardDirection(
                    allianceColor == Alliance.RED
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    /* Log Things */
    SwerveDriveState swerveState = getState();
    DogLog.log("Swerve/Robot Pose", swerveState.Pose);
    DogLog.log("Swerve/Speeds", swerveState.Velocity);
    DogLog.log("Swerve/ModulePositions", swerveState.ModulePositions);
    DogLog.log("Swerve/ModuleTargets", swerveState.ModuleTargets);
    DogLog.log("Swerve/Timestamp", swerveState.Timestamp);
    DogLog.log("Swerve/OdometryFrequency", 1.0 / swerveState.OdometryPeriod);

    DogLog.log("Swerve/FrontLeftDrive Stator Current", frontLeftDriveStatorCurrent.getValueAsDouble());
    //DogLog.log("Swerve/FrontLeftTurn Stator Current", );
    //DogLog.log("Swerve/", );
    //DogLog.log("Swerve/", );
    //DogLog.log("Swerve/", );
    //DogLog.log("Swerve/", );    
  }

  public Command setRotationTarget(RotationTarget rotationTarget) {
    return Commands.runOnce(
        () -> {
          this.rotationTarget = rotationTarget;
        });
  }

  public RotationTarget getRotationTarget() {
    return this.rotationTarget;
  }

  public double getGoalHeading() {
    switch (this.rotationTarget) {
      case FORTY_FIVE:
        return 45.0;
      case NORMAL:
        return 0;
      case RED_HUB:
        Translation2d robotCoordinates = getState().Pose.getTranslation();
        Translation2d redHub = new Translation2d(11.92, 4.03);

        return redHub.minus(robotCoordinates).getAngle().get().getDegrees();
      default:
        return 0;
    }
  }

  public Command setLimitAcceleration(boolean enable) {
    return Commands.runOnce(
        () -> {
          this.isLimitAcceleration = true;
          this.translationSlowFactor = Math.clamp(translationSlowFactor, 0, 1);
          this.rotationSlowFactor = Math.clamp(rotationSlowFactor, 0, 1);
        });
  }

  public boolean isLimitAcceleration() {
    return isLimitAcceleration;
  }

  public Command setSlowMode(boolean enable) {
    return Commands.runOnce(
        () -> {
          this.isSlowMode = enable;
          this.translationSlowFactor = defaultSlowFactor;
          this.rotationSlowFactor = defaultSlowFactor;
        });
  }

  public Command setSlowMode(double translationSlowFactor, double rotationSlowFactor) {
    return Commands.runOnce(
        () -> {
          this.isSlowMode = true;
          this.translationSlowFactor = Math.clamp(translationSlowFactor, 0, 1);
          this.rotationSlowFactor = Math.clamp(rotationSlowFactor, 0, 1);
        });
  }

  public boolean isSlowMode() {
    return isSlowMode;
  }

  public double getTranslationSlowFactor() {
    return translationSlowFactor;
  }

  public double getRotationalSlowFactor() {
    return rotationSlowFactor;
  }
}

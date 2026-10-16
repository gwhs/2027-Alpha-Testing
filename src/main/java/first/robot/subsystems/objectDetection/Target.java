package first.robot.subsystems.objectDetection;

import org.wpilib.math.geometry.Pose2d;

public record Target(double timestamp, Pose2d pose) {}

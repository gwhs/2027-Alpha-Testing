// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

public class RobotContainer {
  public static SwerveDrivetrainConstants controller;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
    
  }
}

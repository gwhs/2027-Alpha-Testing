// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;

/** Add your docs here. */
public enum RobotType {
    DEV,
    COMP,
    KITBOT,
    SIM;


public static RobotType getRobot() {
    String serialNumber = RobotController.getSerialNumber();

    if (RobotBase.isSimulation()) {
        return SIM;
    } else if (serialNumber.equals("12345678")) {
        return DEV;
    } else if (serialNumber.equals("03223849")) {
        return COMP;
    } else if (serialNumber.equals("0323CA18")) {
        return KITBOT;
    } else {
        return COMP;
    }
  }
}

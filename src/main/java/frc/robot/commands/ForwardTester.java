// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ForwardTester extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public ForwardTester(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.45, 30, drivetrain)

        );
        /* 
         * 0.65 = 27.6
         * 0.6 = 27.9
         * 0.55 = 29.55
         * 0.5 = 29.75
         * 0.45 = 29.9
         * 0.4 = 31.6
         * 0.3 = 32.12
         * 0.2 = 0.
         * 0.1 = 0.
         * --------
         * 30 cm
         * 
         */
  }
}

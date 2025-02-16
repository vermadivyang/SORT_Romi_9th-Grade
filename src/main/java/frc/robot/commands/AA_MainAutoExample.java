// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AA_MainAutoExample extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AA_MainAutoExample(Drivetrain drivetrain) {
    Forward(0.5,30,drivetrain);
    Turn(0.4,180,drivetrain);
    Forward(0.5,30,drivetrain);

    /*addCommands(
      new DriveDistance(0.5, 30, drivetrain),
      new TurnDegrees(0.4, 180, drivetrain),
      new DriveDistance(0.5, 30, drivetrain));*/
  }

  public void Forward(double speed, double cm, Drivetrain drivetrain){
    addCommands(
      new DriveDistance(speed, cm, drivetrain));
      Wait(1, drivetrain);
  }
  public void Turn(double speed, double degrees, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees(speed, degrees, drivetrain));
      Wait(1, drivetrain);
  }
  public void Wait(double second, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees(0, second, drivetrain));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AA_CoordAuto extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  
   double cur_pos_x;
   double cur_pos_y;
   double orient;
   double x_Sum;
   double y_Sum;
   double z_Sum;

   double default_speedDistance = 0.5;
   double default_speedAngle = 0.4;
   double default_waitTime = 0.5;

  public AA_CoordAuto(Drivetrain drivetrain) {
    //Init
    StartPos(0.5,3,90);
    Wait(1, drivetrain);

    //Program Here
    moveToForwards(1,3,drivetrain);
    Turn(0.4,180,drivetrain);
    Forward(0.5,25,drivetrain);

    /*addCommands(
      new DriveDistance(0.5, 30, drivetrain),
      new TurnDegrees(0.4, 180, drivetrain),
      new DriveDistance(0.5, 30, drivetrain));*/
  }

  public void Forward(double speed, double cm, Drivetrain drivetrain){
    addCommands(
      new DriveDistance(speed, cm, drivetrain));
      Wait(default_waitTime, drivetrain);
  }
  public void Turn(double speed, double degrees, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees(speed, degrees, drivetrain));
      Wait(default_waitTime, drivetrain);
  }
  public void Wait(double second, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees(0, second, drivetrain));
  }

  public void Wait(Drivetrain drivetrain){
    addCommands(
      new TurnDegrees(0, default_waitTime, drivetrain));
  }

  public void Turn(double degrees, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees( default_speedAngle, degrees, drivetrain));
      Wait(default_waitTime, drivetrain);
  }

  public void Forward(double cm, Drivetrain drivetrain){
    addCommands(
      new DriveDistance(default_speedDistance, cm, drivetrain));
      Wait(default_waitTime, drivetrain);
  }

  public void turn(double degrees, Drivetrain drivetrain){
    addCommands(
      new TurnDegrees( default_speedAngle, degrees, drivetrain));
      Wait(default_waitTime, drivetrain);
  }

  public void forward(double cm, Drivetrain drivetrain){
    addCommands(
      new DriveDistance(default_speedDistance, cm, drivetrain));
      Wait(default_waitTime, drivetrain);
  }

  public void backward(double cm, Drivetrain drivetrain){
    addCommands(
      new DriveDistance(-default_speedDistance, cm, drivetrain));
      Wait(default_waitTime, drivetrain);
  }

  public void StartPos(double sx,double sy,double so){
  
  cur_pos_x = sx;
  cur_pos_y = sy;
  orient = so;
  x_Sum = 0;
  y_Sum = 0;
  z_Sum = 0;
}


public void moveToForwards(double x, double y, Drivetrain drivetrain){

double new_pos_x = x;
double new_pos_y = y;

//Veritcal Movment
if (cur_pos_x == new_pos_x){
  y_Sum = y_Sum + Math.abs(new_pos_y-cur_pos_y);
  if (new_pos_y > cur_pos_y){
    turn(90-orient, drivetrain);
    forward(new_pos_y-cur_pos_y, drivetrain);
    orient=90;
  }
  else if (new_pos_y < cur_pos_y){
    double taa = -90-orient;
    if (Math.abs(taa) > 181){
      taa = 270-orient;}
    turn(taa, drivetrain);
    forward(Math.abs(new_pos_y-cur_pos_y), drivetrain);
    orient=-90;
  }
}

//Horizontal Movment
else if (cur_pos_y == new_pos_y){
  x_Sum = x_Sum + Math.abs(new_pos_x-cur_pos_x);

  if (new_pos_x > cur_pos_x){
    turn(0-orient, drivetrain);
    forward(new_pos_x-cur_pos_x, drivetrain);
    orient=0;
  }
  else if (new_pos_x < cur_pos_x) {
    double ta = 180-orient;
    if (Math.abs(ta) > 181){
      ta = -180-orient;}
    turn(ta, drivetrain);
    forward(Math.abs(new_pos_x-cur_pos_x), drivetrain);
    orient=180;
  }
}  
//Diagonal Movement    
else{
  
 if(new_pos_x > cur_pos_x){
   double dif_x = new_pos_x-cur_pos_x;
   double dif_y = new_pos_y-cur_pos_y;
   double distance = Math.pow((dif_x*dif_x) + (dif_y*dif_y),0.5) ;
   z_Sum = distance + z_Sum;
   double angle = Math.toDegrees(Math.asin(dif_y/distance));
   turn(angle-orient, drivetrain);  
   forward(distance, drivetrain);
   orient=angle;}
    
 else if (new_pos_x < cur_pos_x){
  double dif_x = new_pos_x-cur_pos_x;
  double dif_y = new_pos_y-cur_pos_y;
  double distance = Math.pow((dif_x*dif_x) + (dif_y*dif_y),0.5) ;
   z_Sum = distance + z_Sum;
   double angle = Math.toDegrees(Math.asin(dif_y/distance));
   double angle2 = (180 - (2*angle))+ angle;
   turn(angle2-orient, drivetrain);  
   forward(distance, drivetrain);
   orient=angle2;}
}
    
cur_pos_x = new_pos_x;
cur_pos_y = new_pos_y;
}

public void moveToBackwards(double x, double y, Drivetrain drivetrain){

  double new_pos_x = x;
  double new_pos_y = y;
  
  //Veritcal Movment
  if (cur_pos_x == new_pos_x){
    y_Sum = y_Sum + Math.abs(new_pos_y-cur_pos_y);
    if (new_pos_y > cur_pos_y){
      double taaaa = -90-orient;
      if (Math.abs(taaaa) > 181){
        taaaa = 270-orient;}
      turn(taaaa, drivetrain) ;  
      backward(new_pos_y-cur_pos_y,drivetrain);
      orient=-90;
    }
    else if (new_pos_y < cur_pos_y){
      double taa = 90-orient;
      if (Math.abs(taa) > 181){
        taa = -270-orient;}
      turn(taa, drivetrain);
      forward(Math.abs(new_pos_y-cur_pos_y), drivetrain);
      orient=90;
    }
  }
  
  //Horizontal Movment
  else if (cur_pos_y == new_pos_y){
    x_Sum = x_Sum + Math.abs(new_pos_x-cur_pos_x);
  
    if (new_pos_x > cur_pos_x){
      double taedaa = 180-orient;
      if (Math.abs(taedaa) > 181){
      taedaa = -180-orient;}
      turn(taedaa, drivetrain)   ;
      backward(new_pos_x-cur_pos_x, drivetrain);
      orient=180;   
      
    }
    else if (new_pos_x < cur_pos_x) {
      turn(0-orient, drivetrain);
      forward(Math.abs(new_pos_x-cur_pos_x), drivetrain);
      orient=0;
    }
  }  
  //Diagonal Movement    
  else{
    
   if(new_pos_x < cur_pos_x){
     double dif_x = new_pos_x-cur_pos_x;
     double dif_y = new_pos_y-cur_pos_y;
     double distance = Math.pow((dif_x*dif_x) + (dif_y*dif_y),0.5) ;
     z_Sum = distance + z_Sum;
     double angle = Math.toDegrees(Math.asin(dif_y/distance));
     turn(angle-orient, drivetrain);  
     backward(distance, drivetrain);
     orient=-angle;}
      
   else if (new_pos_x > cur_pos_x){
    double dif_x = new_pos_x-cur_pos_x;
    double dif_y = new_pos_y-cur_pos_y;
    double distance = Math.pow((dif_x*dif_x) + (dif_y*dif_y),0.5) ;
     z_Sum = distance + z_Sum;
     double angle = Math.toDegrees(Math.asin(dif_y/distance));
     double angle2 = (180 - (2*angle))+ angle;
     turn(angle2-orient, drivetrain);  
     backward(distance, drivetrain);
     orient=-angle2;}
  }
      
  cur_pos_x = new_pos_x;
  cur_pos_y = new_pos_y;
  }

}

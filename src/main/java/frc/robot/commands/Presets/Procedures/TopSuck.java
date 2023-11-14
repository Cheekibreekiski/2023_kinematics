// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
//import frc.robot.commands.Presets.RunWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class TopSuck extends ParallelCommandGroup {
  // elevator down, arm down, wrist up, intake untill hard stop

  public TopSuck(Elevator e, Arm a, Intake i, Wrist W) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //always set the wrist to false for everything 
    addCommands(
      // Commands.parallel(new SetElevator(e,1),new SetArm(a,2)) //new RunWrist(W, false))
      
      new SetElevator(e, 0, W),new SetWrist(W, WristConstants.groundPickupWrist), new SetArm(a, ArmConstants.groundPickupArm)
    );
  }
}

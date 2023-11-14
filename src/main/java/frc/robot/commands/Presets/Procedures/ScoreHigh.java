// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
//import frc.robot.commands.Presets.RunWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;



public class ScoreHigh extends SequentialCommandGroup {
    // elevator high, arm out, wrist down, intake out
      public ScoreHigh(Elevator e, Arm a, Intake i, Wrist w) {
        //addRequirements(w);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(//will not work because we need to create the set elevator and arm commands
      // new SetArm(a,1),//set arm to pos 1      
      // Commands.parallel(new SetElevator(e,2)),//new RunWrist(w,false )),
      // new SetArm(a,2)
      Commands.parallel(new SetArm(a, ArmConstants.minNonCollidingExtention), new SetWrist(w, WristConstants.startingPosWrist)),
      new SetElevator(e, 2, w),
      new ParallelCommandGroup(new SetArm(a, ArmConstants.highScoringPos), new SetWrist(w, WristConstants.highScorePos))
    );
  }
}

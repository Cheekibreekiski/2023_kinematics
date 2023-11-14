// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.kinematics.Forward;
import frc.robot.kinematics.Inverse;
import frc.robot.kinematics.KinematicProfile;
import frc.robot.kinematics.SuperstructureState;
import frc.robot.kinematics.Util.Coords;

/** Add your docs here. */
public class KinematicEngine extends SubsystemBase{
    KinematicProfile kp;
    Inverse inv;
    Forward fwd;
    Arm arm;
    Wrist wrist;
    Elevator elevator;
    double theta1Offset = 0;
    double theta2Offset = 0;

    public KinematicEngine(KinematicProfile kp, Arm arm, Wrist wrist, Elevator elevator){
        this.kp = kp;
        fwd = new Forward(kp);
        inv = new Inverse(kp);
        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
    }

    public SuperstructureState calcInverse(Coords desiredState){
        return inv.calculate(desiredState.getX(), desiredState.getY());
    }

    public Coords calcForward(SuperstructureState currentState){
        return fwd.calculate(
            currentState.getTheta1(),
            currentState.getTheta2(),
            currentState.getElevatorState()
        );
    }

    //degrees
    public double gettheta1(){
        double x = arm.getAbsEncoderPos() - theta1Offset;
        x = x/Constants.CANCODER_CPR;
        return x*360;
    }

    //degrees
    public double gettheta2(){
        double x = arm.getAbsEncoderPos() - theta1Offset;
        x = x/Constants.CANCODER_CPR;
        return x*360;
    }

    //inches?
    public double getElevatorPos(){
        double x = elevator.getEncoder();
        x = Conversions.falconToDegrees(x, Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
        return 0;
    }


    public Coords getCurrentPos(){
        return fwd.calculate(gettheta1(), gettheta2(), getElevatorPos());
    }

    public SuperstructureState getCurrentState(){
        return null;
    }
    
    
    
}

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


    /*TODO: test methods in software and on robot
      
        SOFTWARE
        getTheta1: UNTESTED
        getTheta2: UNTESTED
        getElevatorState: UNTESTED
        getElevatorCoords: UNTESTED
     */

    public KinematicEngine(KinematicProfile kp, Arm arm, Wrist wrist, Elevator elevator){
        this.kp = kp;
        fwd = new Forward(kp);
        inv = new Inverse(kp);
        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
        theta1Offset = kp.theta1Offset;
        theta2Offset = kp.theta2Offset;
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
        double x = arm.getAbsEncoderPos() - theta1Offset; //get angle, adjusted to standard pos
        x = x/Constants.CANCODER_CPR; //convert to scale factor
        return x*360; //convert to degrees
    }

    //degrees
    public double gettheta2(){
        double x = wrist.getAbsEncoderPos() - theta2Offset; 
        x = x/Constants.CANCODER_CPR;
        return x*360;
    }

    //meters?
    public double getElevatorState(){
        double x = elevator.getEncoder();
        x = x/Constants.ElevatorConstants.topLimit;
        return x*Constants.ElevatorConstants.ELEVATOR_LENGTH_METERS;
    }

    public Coords getElevatorCoords(){
        return Inverse.getElevatorPos(kp, getElevatorState());
    }



    public Coords getCurrentPos(){
        return fwd.calculate(gettheta1(), gettheta2(), getElevatorState());
    }

    public SuperstructureState getCurrentState(){
        return new SuperstructureState(
            getElevatorState(),
            getElevatorCoords().getX(),
            getElevatorCoords().getY(),
            gettheta1(),
            gettheta2()
        );
    }
    
    
    
}

package frc.robot.kinematics;

public class SuperstructureState {
    double elevatorState;
    double theta1State;
    double theta2State;
    double elevatorX; 
    double elevatorY;

    public SuperstructureState(double elevatorState, double elevatorX, double elevatorY, double theta1State, double theta2State){
        this.elevatorState = elevatorState; 
        this.theta1State = theta1State;
        this.theta2State = theta2State;
        this.elevatorX = elevatorX;
        this.elevatorY = elevatorY;
        }
    
    public double getElevatorState(){
        return elevatorState;
    }
    public double getTheta1(){
        return theta1State;
    }
    public double getTheta2(){
        return theta2State;
    }
    public void setElevatorState(double elevatorState){
        this.elevatorState = elevatorState;
    }
    public void setTheta1State(double theta1State){
        this.theta1State = theta1State;
    }
    public void setTheta2State(double theta2State){
        this.theta2State = theta2State;
    }
    /*
     * [elevatorState, theta1State, theta2State]
     */
    public double[] getArray(){
        double[] arr = {elevatorState, elevatorX, elevatorY, theta1State, theta2State};
        return arr;
    }

    public String toString(){
        return "Elevator State: " + elevatorState + "\n" + "Elevator x: " + elevatorX + "\n" + "Elevator y: " + elevatorY + "\n" + "Theta 1 State: " + theta1State + "\n" + "Theta 2 State: " + theta2State;
    }
}


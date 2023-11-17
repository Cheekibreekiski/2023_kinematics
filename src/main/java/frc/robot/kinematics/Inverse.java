package frc.robot.kinematics;

import frc.robot.kinematics.Util.Coords;

public class Inverse {
    double h = 0;
    double x_e = 0;
    double y_e = 0;
    double x_desired;
    double y_desired;
    KinematicProfile kp;
    double y_elevator_adjusted;
    double x_elevator_adjusted;

    /*
     * @param x_d desired x pos
     * @param y_d desired y pos
     */
    public Inverse(KinematicProfile kp){
        this.kp = kp;
    }





    public SuperstructureState calculate(double x_d, double y_d){        

        x_desired = x_d;
        y_desired = y_d;
        
        //calc elevator pos

        //calculate elevator y
        y_elevator_adjusted = 0; //desired y minus the elevator position
        double elevator_y = 0; //actual y of the elevator
        
        double elevator_extention = y_desired/Math.sin(kp.theta_e);
        double max_elevator_y = kp.e_max*Math.sin(kp.theta_e);

        
        
        //TODO: change elevator behavior
        
        //if desired y is greater than max extension allows, bring the elevator to max
        if(elevator_extention > kp.e_max){
            y_elevator_adjusted = y_desired - max_elevator_y;
            elevator_y = max_elevator_y;

        //if desired y is within range of the elevator, bring the elevator to the desired y
        }else if(elevator_extention >= 0 && elevator_extention <= kp.e_max){
            y_elevator_adjusted = 0;
            elevator_y = y_desired;

        //if desired y is less than 0, bring the elevator to 0
        }else{
            y_elevator_adjusted = y_desired;
            elevator_y = 0;
        }

        //calc elevator x
        double elevator_x = elevator_y/Math.tan((kp.theta_e));
        x_elevator_adjusted = x_desired - elevator_x;

        //calc h, used in later calculations
        //double h = Math.sqrt((x_elevator_adjusted*x_elevator_adjusted+(y_elevator_adjusted*y_elevator_adjusted)));
        double h2 = (x_elevator_adjusted*x_elevator_adjusted)+(y_elevator_adjusted*y_elevator_adjusted);
    
        
        
        //calc theta 2
        double theta2 = Math.acos(((h2)-(kp.l1*kp.l1)-(kp.l2*kp.l2))/(-2*kp.l1*kp.l2));


        double theta1 = 0;
        //calc theta 1
        if(theta2 > 0){
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) - Math.atan((kp.l2*Math.sin(theta2))/(kp.l1+(kp.l2*Math.cos(theta2))));
        }else{
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) + Math.atan((kp.l2*Math.sin(theta2))/(kp.l1+(kp.l2*Math.cos(theta2))));
        }
        
        return new SuperstructureState(
            elevator_extention, 
            elevator_x,
            elevator_y, 
            Math.toDegrees(theta1)+90, 
            Math.toDegrees(theta2)-180
            );
        
    }
    
    public static Coords getElevatorPos(KinematicProfile kp, double extention){
        return new Coords(
            extention*Math.cos(kp.theta_e), 
            extention*Math.sin(kp.theta_e)
        );
    }
}

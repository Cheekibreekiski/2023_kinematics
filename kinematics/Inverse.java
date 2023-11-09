package kinematics;
import java.util.function.DoubleSupplier;

import kinematics.Util.Coords;

//import Robot class


class Inverse {
    Coords desired_coords;
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
    public Inverse(DoubleSupplier x_d, DoubleSupplier y_d, KinematicProfile kp){
        
        desired_coords = new Coords(x_d.getAsDouble(), y_d.getAsDouble());
        this.kp = kp;
    }





    public RobotState calculate(double x_d, double y_d){        

        x_desired = x_d;
        y_desired = y_d;
        

        //calc elevator pos

        //calculate elevator y
        y_elevator_adjusted = 0; //desired y minus the elevator position
        double elevator_y = 0; //actual y of the elevator
        
        double elevator_extention = y_desired/Math.sin(kp.theta_e);
        double max_elevator_y = kp.e_max*Math.sin(kp.theta_e);

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

        //calc elevator extention
        double e = elevator_y/Math.sin(kp.theta_e);

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
        
        return new RobotState(e, elevator_x, elevator_y, Math.toDegrees(theta1)+90, Math.toDegrees(theta2)-180);
        
    }

    /* 
    public RobotState arm(double x_d, double y_d){
        double x_desired = x_d;
        double y_desired = y_d;
        
        double x_elevator_adjusted = x_desired;
        double y_elevator_adjusted = y_desired;
        
        //calc h, used in later calculations
        //double h = Math.sqrt((x_elevator_adjusted*x_elevator_adjusted+(y_elevator_adjusted*y_elevator_adjusted)));
        double h2 = (x_elevator_adjusted*x_elevator_adjusted)+(y_elevator_adjusted*y_elevator_adjusted);
    
        
        
        //calc theta 2
        double theta2 = Math.acos(((h2)-(kp.l1*kp.l1)-(kp.l2*kp.l2))/(-2*kp.l1*kp.l2));

        //calc theta 1
        double theta1 = 0;
        if(theta2 > 0){
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) - Math.atan((kp.l2*Math.sin(theta2))/(kp.l1+(kp.l2*Math.cos(theta2))));
        }else{
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) + Math.atan((kp.l2*Math.sin(theta2))/(kp.l1+(kp.l2*Math.cos(theta2))));
        }
        
        //im not entirely sure why we offset the values these specific amounts, but this aligns everything to the desired angles
        return new RobotState(0, 0, 0, 90-Math.toDegrees(theta1), Math.toDegrees(theta2)-180);


    }
    */
}

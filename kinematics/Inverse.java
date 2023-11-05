package kinematics;
import java.util.function.DoubleSupplier;

import kinematics.Util.Coords;

//import Robot class


class Inverse extends KinematicProfile{
    Coords desired_coords;
    double h = 0;
    double x_e = 0;
    double y_e = 0;
    DoubleSupplier x_d;
    DoubleSupplier y_d;

    /*
     * @param x_d desired x pos
     * @param y_d desired y pos
     */
    public Inverse(DoubleSupplier x_d, DoubleSupplier y_d, double l1, double l2, double theta_e, double e_max){
        super(l1, l2, theta_e, e_max);
        desired_coords = new Coords(x_d.getAsDouble(), y_d.getAsDouble());
        this.x_d = x_d;
        this.y_d = y_d;
    }





    public RobotState calculate(){        

        double x_desired = desired_coords.getX();
        double y_desired = desired_coords.getY();
        

        //calc elevator pos

        //calculate elevator y
        double y_elevator_adjusted = 0; //desired y minus the elevator position
        double elevator_y = 0; //actual y of the elevator
        
        //if desired y is greater than max extension, bring the elevator to max
        if(y_desired > e_max){
            y_elevator_adjusted = y_desired - e_max;
            elevator_y = e_max;

        //if desired y is less than 0, bring the elevator to 0
        }else if(y_desired >= 0 && y_desired <= e_max){
            y_elevator_adjusted = 0;
            elevator_y = y_desired;
        //if desired y is within range of the elevator, bring the elevator to the desired y
        }else{
            y_elevator_adjusted = y_desired;
            elevator_y = 0;
        }

        //calc elevator x
        double elevator_x = elevator_y/Math.tan((theta_e));
        double x_elevator_adjusted = x_desired - elevator_x;

        //calc elevator extention
        double e = elevator_y/Math.sin(theta_e);

        //calc h, used in later calculations
        //double h = Math.sqrt((x_elevator_adjusted*x_elevator_adjusted+(y_elevator_adjusted*y_elevator_adjusted)));
        double h2 = (x_elevator_adjusted*x_elevator_adjusted)+(y_elevator_adjusted*y_elevator_adjusted);
    
        
        
        //calc theta 2
        double theta2 = Math.acos(((h2)-(l1*l1)-(l2*l2))/(-2*l1*l2));


        double theta1 = 0;
        //calc theta 1
        if(theta2 > 0){
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) - Math.atan((l2*Math.sin(theta2))/(l1+(l2*Math.cos(theta2))));
        }else{
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) + Math.atan((l2*Math.sin(theta2))/(l1+(l2*Math.cos(theta2))));
        }
        
        return new RobotState(e, elevator_x, elevator_y, Math.toDegrees(theta1), Math.toDegrees(theta2));
        
    }

    public RobotState arm(double x_d, double y_d){
        double x_desired = x_d;
        double y_desired = y_d;
        
        double x_elevator_adjusted = x_desired;
        double y_elevator_adjusted = y_desired;
        
        //calc h, used in later calculations
        //double h = Math.sqrt((x_elevator_adjusted*x_elevator_adjusted+(y_elevator_adjusted*y_elevator_adjusted)));
        double h2 = (x_elevator_adjusted*x_elevator_adjusted)+(y_elevator_adjusted*y_elevator_adjusted);
    
        
        
        //calc theta 2
        double theta2 = Math.acos(((h2)-(l1*l1)-(l2*l2))/(-2*l1*l2));


        double theta1 = 0;
        //calc theta 1
        if(theta2 > 0){
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) - Math.atan((l2*Math.sin(theta2))/(l1+(l2*Math.cos(theta2))));
        }else{
            theta1 = Math.atan(y_elevator_adjusted/x_elevator_adjusted) + Math.atan((l2*Math.sin(theta2))/(l1+(l2*Math.cos(theta2))));
        }
        
        return new RobotState(0, 0, 0, 90-Math.toDegrees(theta1), Math.toDegrees(theta2)-180);


    }
}

package kinematics;

import kinematics.Util.Coords;

public class Forward {
    double l1;
    double l2;
    double theta_e;
    

    public Forward(KinematicProfile kp){
        this.l1 = kp.l1;
        this.l2 = kp.l2;
        this.theta_e = kp.theta_e;
        
    }

    /*
     * in degrees
     */
    public Coords calculate(double theta1, double theta2, double e){
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(180 - theta2);
        double x = l1*Math.cos(theta1) + l2*Math.cos(theta2) + e*Math.cos(theta_e);
        double y = l1*Math.sin(theta1) + l2*Math.sin(theta2) + e*Math.sin(theta_e);
        
        //round
        x = (x%0.01);
        y = y - (y%0.01);

        

        return new Coords(x, y);
    }
}

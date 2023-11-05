package kinematics;

import kinematics.Util.Coords;

public class Forward {
    double l1;
    double l2;
    double theta_e;
    

    public Forward(double l1, double l2, double theta_e){
        this.l1 = l1;
        this.l2 = l2;
        this.theta_e = theta_e;
        
    }

    /*
     * in degrees
     */
    public Coords calculate(double theta1, double theta2, double e){
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        theta_e = Math.toRadians(theta_e);
        double x = l1*Math.cos(theta1) + l2*Math.cos(theta1+theta2) + e*Math.cos(theta_e);
        double y = l1*Math.sin(theta1) + l2*Math.sin(theta1+theta2) + e*Math.sin(theta_e);
        return new Coords(x, y);
    }
}
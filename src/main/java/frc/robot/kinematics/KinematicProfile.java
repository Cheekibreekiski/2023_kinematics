package frc.robot.kinematics;
//constants and stuff


import frc.robot.kinematics.Util.Coords;

public class KinematicProfile{
    public double l1; //link lengths
    public double l2;
    public double theta_e; //elevator angle
    public double e_max; //max elevator extention
    public Coords des;
    public double theta1Offset; //encoder offsets
    public double theta2Offset;
    
    /**
     * all distance units are in meters
     * all angle mesures are input as degrees, and STORED AS RADIANS!!!!!!
     * @param l1 length of link 1
     * @param l2 length of link 2
     * @param theta_e elevator angle in degrees
     * @param e_max max elevator extention
     * 
     */
    public KinematicProfile(double l1, double l2, double theta_e, double e_max, double theta1Offset, double theta2Offset){
        this.l1 = l1;
        this.l2 = l2;
        this.e_max = e_max;
        this.theta_e = Math.toRadians(theta_e);
        this.theta1Offset = theta1Offset;
        this.theta2Offset = theta2Offset;
    }
    

    
}

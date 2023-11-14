package frc.robot.kinematics;
//constants and stuff


import frc.robot.kinematics.Util.Coords;

public class KinematicProfile{
    double l1; //link lengths
    double l2;
    double theta_e; //elevator angle
    double e_max; //max elevator extention
    public Coords des;
    
    /*
     * @param l1 length of link 1
     * @param l2 length of link 2
     * @param theta_e elevator angle in degrees
     */
    public KinematicProfile(double l1, double l2, double theta_e, double e_max){
        this.l1 = l1;
        this.l2 = l2;
        this.e_max = e_max;
        this.theta_e = Math.toRadians(theta_e); 
    }
    

    
}

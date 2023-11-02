package kinematics;

import java.util.function.DoubleSupplier;

import kinematics.Robot.coords;

class Main{
    

    public static void main(String[] args){
        DoubleSupplier x_d = (() -> 2);
        DoubleSupplier y_d = (() -> 2);
        double l1 = 1.0;
        double l2 = 1.0;
        double theta_e = 45.0;
        double e_max = 5.0;
        coords des;
        
        inverse inv = new inverse(x_d, y_d, l1, l2, theta_e, e_max);

        System.out.println(inv.calculate().toString());
        
    }
}
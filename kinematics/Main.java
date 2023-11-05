package kinematics;

import java.util.function.DoubleSupplier;


class Main{
    

    public static void main(String[] args){
        
        
        
        DoubleSupplier x_d = (() -> 4.5);
        DoubleSupplier y_d = (() -> 2);
        double l1 = 1.0;
        double l2 = 1.0;
        double theta_e = 45.0;
        double e_max = 5.0;
        
        
        Inverse inv = new Inverse(x_d, y_d, l1, l2, theta_e, e_max);
        Forward fwd = new Forward(l1, l2, theta_e);

        // 
        // System.out.println(calc.toString());
        // System.out.println(fwd.calculate(calc.getTheta1(),calc.getTheta2(), calc.getElevatorState()).toString());

        RobotState calc = inv.arm(0, -2);
        System.out.println(calc.toString());
        System.out.println(fwd.calculate(calc.getTheta1(), calc.getTheta2(), 0));

    }
}
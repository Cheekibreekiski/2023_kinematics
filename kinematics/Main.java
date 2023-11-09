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
        KinematicProfile kp = new KinematicProfile(l1, l2, theta_e, e_max);
        
        //TODO: goes beyond max elevator state
        //TODO: possibly rederive foward kinematics
        
        Inverse inv = new Inverse(x_d, y_d, kp);
        Forward fwd = new Forward(kp);

        RobotState calc = inv.calculate(2, 1);
        
        if(!((inv.x_elevator_adjusted*inv.x_elevator_adjusted)+(inv.y_elevator_adjusted*inv.y_elevator_adjusted)<=4)){
           System.out.println("\n //////////////////////////// Outside of work packet! ///////////////////////////// \n");
        }
            System.out.println();
            System.out.println("Inverse Kinematics:");
            System.out.println(calc.toString());
            System.out.println("Calculated coords");
            System.out.println(fwd.calculate(calc.getTheta1(), calc.getTheta2(), calc.getElevatorState()));
            System.out.println("Adjusted coords:");
            System.out.println("(" + inv.x_elevator_adjusted + "," + inv.y_elevator_adjusted + ")");
            System.out.println("Actual coords:");
            System.out.println("(" + inv.x_desired + "," + inv.y_desired + ")");
            System.out.println();
        
         
        
    }
}
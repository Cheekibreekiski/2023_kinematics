//constants and stuff

import java.util.function.DoubleSupplier;

class KinematicObject{
    double l1; //link lengths
    double l2;
    double theta_e; //elevator angle
    double e_max; //max elevator extention
    

    public KinematicObject(double l1, double l2, double theta_e){
        this.l1 = l1;
        this.l2 = l2;
        this.theta_e = theta_e; 
    }

    public class coords{
        double x;
        double y;
        public coords(double x, double y){
            this.x = x;
            this.y = y;
        }
    
        public double getX(){
            return x;
        }
        public double getY(){
            return y;
        }
        public void setX(double x){
            this.x = x;
        }
        public void setY(double y){
            this.y = y;
        }
        /*
         * [x,y]
         */
        public double[] getArray(){
            double[] arr = {x, y};
            return arr;
        }
    }
}
import java.util.function.DoubleSupplier;

import KinematicObject.coords;

class inverse extends KinematicObject{
    coords des;
    double h;
    double x_e;
    double y_e;

    /*
     * @param x_d desired x pos
     * @param y_d desired y pos
     */
    public inverse(DoubleSupplier x_d, DoubleSupplier y_d, double l1, double l2, double theta_e){
        super(l1, l2, theta_e);
        des = new coords(x_d, y_d);
    }

    public void updateCoords(){
        coords.setX(x_d.get());
        coords.setY(y_d.get());
        h = Math.sqrt((x_e*x_e)+(x_e*x_e));
    }

    /*
     * returns the position of the elevator
     */
    public void getElevator(){
        double pos;
        double y;
        double x;
        updateCoords();
        
        //calculate y
        if(y_d > e_max){
            y = e_max;
        }else if(y_d >= 0 && y_d <= e_max){
            y = y_d;
        }else{
            y = 0;
        }

        //calculate x
        

    }

    public void getTheta1(){
        updateCoords();
        double res;
        res = Math.acos((l2*l2)-(l1*l1)-(h*h));
    }

}


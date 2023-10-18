package kinematics;
import java.util.function.DoubleSupplier;

//import Robot class


class inverse extends Robot{
    coords des;
    double h;
    double x_e = 0;
    double y_e = 0;
    DoubleSupplier x_d;
    DoubleSupplier y_d;

    /*
     * @param x_d desired x pos
     * @param y_d desired y pos
     */
    public inverse(DoubleSupplier x_d, DoubleSupplier y_d, double l1, double l2, double theta_e){
        super(l1, l2, theta_e);
        des = new coords(x_d.getAsDouble(), y_d.getAsDouble());
        this.x_d = x_d;
        this.y_d = y_d;
    }

    public void updateCoords(){
        des.setX(x_d.getAsDouble());
        des.setY(y_d.getAsDouble());
        h = Math.sqrt((x_e*x_e)+(x_e*x_e));
    }

    /*
     * adjusts coords to account for the elevator position
     */
    public coords adjustCoords(coords des){
        updateCoords();
        double pos;
        double y = 0;
        double x = 0;
        
        //calculate y
        if(des.getY() > e_max){
            y = des.getY() - e_max;
        }else if(des.getY() >= 0 && des.getY() <= e_max){
            y = 0;
        }else{
            y = des.getY();
        }

        //calculate x
        x = des.getX()-(y/Math.tan(theta_e));
        
        return new coords(x, y);
    }

    public void getTheta1(){
        updateCoords();
        double res;
        res = Math.acos((l2*l2)-(l1*l1)-(h*h));
    }

}


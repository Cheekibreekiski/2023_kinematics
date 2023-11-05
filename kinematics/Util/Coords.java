package kinematics.Util;

public class Coords{

        double x;
        double y;
        public Coords(double x, double y){
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
        public double[] toArray(){
            double[] arr = {x, y};
            return arr;
        }

        @Override
        public String toString(){
            return "(" + x + "," + y + ")";
        }
    }

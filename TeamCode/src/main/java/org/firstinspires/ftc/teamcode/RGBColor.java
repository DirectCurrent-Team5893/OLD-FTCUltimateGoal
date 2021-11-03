package org.firstinspires.ftc.teamcode;


public class RGBColor {
    private int red = 0;
    private int green = 0;
    private int blue = 0;

    public RGBColor(){

    }
    public RGBColor(int r, int g, int b){
        red = r;
        green = g;
        blue=  b;
    }

    public void setRed(int r)
    {
        red = r;
    }
    public void setGreen(int g) {
        green = g;
    }
    public void setBlue(int b)
    {
        blue = b;
    }

    /**
     * Returns the value of red
     * @return red
     */
    public int getRed(){
        return red;
    }
    public int getGreen(){
        return green;
    }
    public int getBlue(){
        return blue;
    }
    public int getYellow() {return green + blue;}
    public int getValueA() {return red + green + blue;}
    public int getValueB(){return green + blue - red;}
}

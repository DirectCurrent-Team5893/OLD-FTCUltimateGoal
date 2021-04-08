package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class robotBase {

    protected Telemetry telemetry;
    protected HardwareMapper mapper;
    protected LinearOpMode opMode;
    public HardwareMap hardwaremap;
    protected boolean debugging;

     public robotBase(Telemetry telemetry, LinearOpMode opMode, HardwareMap hardwaremap, boolean debugging){
        this.telemetry = telemetry;
         this.mapper = new HardwareMapper(this);
        this.opMode = opMode;
        this.hardwaremap = hardwaremap;
        this.debugging = debugging;

    }

    public void setDebugging(boolean debug){ debugging = debug;}
    public HardwareMap getHardwaremap() {
        return hardwaremap;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
    public HardwareMapper getMapper(){
        return mapper;
    }
    public void debugTelemetery(String message){
         if(debugging){
             this.getTelemetry().addLine(message);
             this.getTelemetry().update();
         }
    }
    public void debugTelemetery(String message,boolean wait){
        if(debugging){
            this.getTelemetry().addLine(message);
            this.getTelemetry().update();
            this.opMode.sleep(2000);
        }
    }
    public void debugWait(){
        if(debugging) this.opMode.sleep(2000);
    }

    public void init(){}

    abstract public void stop();

}

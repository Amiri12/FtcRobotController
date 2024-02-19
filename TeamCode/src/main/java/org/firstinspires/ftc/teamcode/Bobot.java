package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




public class Bobot {
    
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private int posLF;
    private int posRF;
    private int posLB;
    private int posRB;
    private int pos;

    private int runs;
    
    public Bobot(DcMotor lfd, DcMotor rfd, DcMotor lbd, DcMotor rbd){
        LFD = lfd;
        RFD = rfd;
        LBD = lbd;
        RBD = rbd;
        posLF = 0;
        posRF = 0;
        posLB = 0;
        posRB = 0;
        pos = 0;
        runs = 0;
        LFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFD.setDirection(DcMotor.Direction.FORWARD);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.REVERSE);
        RBD.setDirection(DcMotor.Direction.FORWARD);
    }
    
    public void forward (int dist, double pow){
        posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
        
        LFD.setTargetPosition(posLF + dist);
        
        RFD.setTargetPosition(posRF + dist);
        
        LBD.setTargetPosition(posLB + dist);
        
        RBD.setTargetPosition(posRB + dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runs = 0;
        while(posLF<LFD.getTargetPosition()&&posRF<RFD.getTargetPosition()&&posLB<LBD.getTargetPosition()&&posRB<RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;
            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();

            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }
            
        }
    }
    
    public void forward(double pow){
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
    }
    
    public void stop(){
        LFD.setPower(0.0);
        RFD.setPower(0.0);
        LBD.setPower(0.0);
        RBD.setPower(0.0);
    }
    public void left(double pow){
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setPower(pow);
        RFD.setPower(-pow);
        LBD.setPower(-pow);
        RBD.setPower(pow);
    }
    public void left(int dist, double pow){
        posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        LFD.setPower(pow*mod);
        RFD.setPower(pow*mod);
        LBD.setPower(pow*mod);
        RBD.setPower(pow*mod);
        
        LFD.setTargetPosition(posLF - dist);
        
        RFD.setTargetPosition(posRF + dist);
        
        LBD.setTargetPosition(posLB + dist);
        
        RBD.setTargetPosition(posRB - dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(posLF>LFD.getTargetPosition()&&posRF<RFD.getTargetPosition()&&posLB<LBD.getTargetPosition()&&posRB>RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;
            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();
            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }




        }
    }
    public void right(double pow){
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setPower(-pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(-pow);
    }
    public void right(int dist, double pow){
        posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
        
        LFD.setTargetPosition(posLF + dist);
        
        RFD.setTargetPosition(posRF - dist);
        
        LBD.setTargetPosition(posLB - dist);
        
        RBD.setTargetPosition(posRB + dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(posLF<LFD.getTargetPosition()&&posRF>RFD.getTargetPosition()&&posLB>LBD.getTargetPosition()&&posRB<RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;

            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();
            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }
            
        }
    }
    public void backwards(double pow){
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setPower(-pow);
        RFD.setPower(-pow);
        LBD.setPower(-pow);
        RBD.setPower(-pow);
    }
    public void backwards(int dist, double pow){
       posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
        
        LFD.setTargetPosition(posLF - dist);
        
        RFD.setTargetPosition(posRF - dist);
        
        LBD.setTargetPosition(posLB - dist);
        
        RBD.setTargetPosition(posRB - dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(posLF>LFD.getTargetPosition()&&posRF>RFD.getTargetPosition()&&posLB>LBD.getTargetPosition()&&posRB>RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;

            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();
            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }
        }

    }
    public void rightTurn(int dist , double pow){
        posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
        
        LFD.setTargetPosition(posLF + dist);
        
        RFD.setTargetPosition(posRF - dist);
        
        LBD.setTargetPosition(posLB + dist);
        
        RBD.setTargetPosition(posRB - dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(posLF<LFD.getTargetPosition()&&posRF>RFD.getTargetPosition()&&posLB<LBD.getTargetPosition()&&posRB>RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;

            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();
            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }
        }

    }
    public void leftTurn(int dist , double pow){
        posLF = LFD.getCurrentPosition();
        posRF = RFD.getCurrentPosition();
        posLB = LBD.getCurrentPosition();
        posRB = RBD.getCurrentPosition();
        runs = 0;
        int oldAv; // old average change
        double mod = 0.3;
        
        
        LFD.setPower(pow);
        RFD.setPower(pow);
        LBD.setPower(pow);
        RBD.setPower(pow);
        
        LFD.setTargetPosition(posLF - dist);
        
        RFD.setTargetPosition(posRF + dist);
        
        LBD.setTargetPosition(posLB - dist);
        
        RBD.setTargetPosition(posRB + dist);
        
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(posLF>LFD.getTargetPosition()&&posRF<RFD.getTargetPosition()&&posLB>LBD.getTargetPosition()&&posRB<RBD.getTargetPosition()){
            runs++;
            oldAv = (Math.abs(LFD.getCurrentPosition()-posLF ) + Math.abs(LBD.getCurrentPosition() - posLB) + Math.abs(RBD.getCurrentPosition() - posRB) + Math.abs(RFD.getCurrentPosition() - posRF))/4;

            posLF = LFD.getCurrentPosition();
            posRF = RFD.getCurrentPosition();
            posLB = LBD.getCurrentPosition();
            posRB = RBD.getCurrentPosition();
            if(oldAv < dist/4){

                mod += 0.05;
                mod = Math.min(mod, 1.0);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else if(oldAv > dist/2){
                //pow *= mod;
                mod -= 0.03;
                mod = Math.max(mod,0.1);

                LFD.setPower(pow*mod);
                RFD.setPower(pow*mod);
                LBD.setPower(pow*mod);
                RBD.setPower(pow*mod);
            }else{

                LFD.setPower(pow);
                RFD.setPower(pow);
                LBD.setPower(pow);
                RBD.setPower(pow);
            }
            if(runs > 100){
                break;
            }
        }

    }

    public int GetRuns(){
        int runs1 = runs;
        return runs1;
    }
    
    
}
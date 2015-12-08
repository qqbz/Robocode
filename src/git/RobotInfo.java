/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package git;
import java.awt.geom.*;

/**
 *
 * @author SimonLaptop
 */
public class RobotInfo extends Point2D.Double{
    
    double energy;
    boolean isAlive;
    double velocity;
    double absHeadingRad;
    boolean isTeammate;
    double bearingRad;
    double headingRad;
    
    public void setEnergy(double a){
        this.energy = a;
    }
    
    public double getEnergy(){
        return this.energy;
    }
    
    public void setIsAlive(boolean a){
        this.isAlive = a;
    }
    
    public boolean getIsAlive(){
        return this.isAlive;
    }
    
    public void setVelocity(double a){
        this.velocity = a;
    }
    
    public double getVelocity(){
        return this.velocity;
    }
    
    public void setAbsHeadingRad(double a){
        this.absHeadingRad = a;
    }
    
    public double getAbsHeadingRad(){
        return this.absHeadingRad;
    }
    
    public void setIsTeammate(boolean a){
        this.isTeammate = a;
    }
    
    public boolean getIsTeammate(){
        return this.isTeammate;
    }
    
    public void setBearingRad(double a){
        this.bearingRad = a;
    }
    
    public double getBearingRad(){
        return this.bearingRad;
    }
    
    public void setHeadingRad(double a){
        this.headingRad = a;
    }
    
    public double getHeadingRad(){
        return this.headingRad;
    }
}

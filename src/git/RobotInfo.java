/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package git;

import java.awt.geom.*;

/**
 * RobotInfo-Klasse zum Speichern der Daten der Bots
 *
 * @author Simon Fella
 */
public class RobotInfo extends Point2D.Double {

    private double energy;
    private double velocity;
    private double bearingRad;
    private double headingRad;

    /**
     * Sets the energy-Value.
     * 
     * @param newEnergy 
     */
    public void setEnergy(double newEnergy) {
        this.energy = newEnergy;
    }

    /**
     * Gets the saved energy-Value.
     * 
     * @return the saved energy-Value
     */
    public double getEnergy() {
        return this.energy;
    }

    /**
     * Sets the velocity-Value.
     * 
     * @param newVelocity 
     */
    public void setVelocity(double newVelocity) {
        this.velocity = newVelocity;
    }

    /**
     * Gets the saved velocity-Value.
     * 
     * @return the saved velocity-Value
     */
    public double getVelocity() {
        return this.velocity;
    }

    /**
     * Sets the bearing-Value in rad.
     * 
     * @param newBearingRad 
     */
    public void setBearingRad(double newBearingRad) {
        this.bearingRad = newBearingRad;
    }

    /**
     * Gets the saved bearing-Value in rad.
     * 
     * @return the saved bearing-Value in rad
     */
    public double getBearingRad() {
        return this.bearingRad;
    }

    /**
     * Sets the heading-Value in rad.
     * 
     * @param newHeadingRad 
     */
    public void setHeadingRad(double newHeadingRad) {
        this.headingRad = newHeadingRad;
    }

    /**
     * Get the saved heading-Value in rad.
     * 
     * @return the saved heading-Value in rad
     */
    public double getHeadingRad() {
        return this.headingRad;
    }
}

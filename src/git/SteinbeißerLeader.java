package git;

import robocode.ScannedRobotEvent;
import robocode.TeamRobot;

import java.awt.*;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.Hashtable;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import robocode.MessageEvent;
import robocode.Rules;
import robocode.util.Utils;

public class SteinbeißerLeader extends TeamRobot {

    Hashtable robots;
    String target;
    boolean soloMode = false;
    Point2D goal;

    /*
     onScanned: table updaten
     onDeath: broadcasten
     onMessage: in soloMode umschalten - Umsetzung später
    
     normaler Ablauf:
     eigenen Werte updaten
     scannen
     bewegungsziel berechnen
     schussziel berechnen
     bewegen
     zielen
     schießen
    
     einmalige Sachen:
     Farbe                                       -----
     sich zu Liste hinzufügen                    -----
     */
    public void run() {
        RobotColors c = new RobotColors();
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.radarColor = Color.green;
        c.scanColor = Color.green;
        c.bulletColor = Color.green;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setRadarColor(c.radarColor);
        setScanColor(c.scanColor);
        setBulletColor(c.bulletColor);

        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);

        RobotInfo leader = new RobotInfo();
        robots.put("SteinbeißerLeader", leader);
        leader.setIsAlive(true);
        leader.setIsTeammate(true);

        RobotInfo droid = new RobotInfo();
        robots.put("SteinbeißerDroid", droid);
        droid.setIsAlive(true);
        droid.setIsTeammate(true);

        do {
            leader = (RobotInfo) robots.get("SteinbeißerLeader");
            leader.setLocation(this.getX(), this.getY());
            leader.setEnergy(this.getEnergy());
            leader.setAbsHeadingRad(this.getHeading());
            leader.setVelocity(this.getVelocity());

            doRadar();
            aimAndShoot();
            try {
                broadcastMessage(robots);
                broadcastMessage(target);
            } catch (IOException ex) {
                Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
            }
            move();
            execute();
        } while (true);

    }

    public void onScannedRobot(ScannedRobotEvent e) { //angelehnt an Shiz
        String scanName = e.getName();
        RobotInfo robot = (RobotInfo) robots.get(scanName);
        if (robot == null) {
            robot = new RobotInfo();
            robots.put(scanName, robot);
        }
        robot.energy = e.getEnergy();
        robot.isAlive = true;
        robot.isTeammate = isTeammate(e.getName());
        robot.absHeadingRad = e.getBearingRadians() + getHeadingRadians();
        robot.velocity = e.getVelocity();
        RobotInfo leader = (RobotInfo) robots.get("SteinbeißerLeader");
        robot.setLocation(projectPoint(new Point2D.Double(leader.getX(), leader.getY()), getHeadingRadians() + e.getBearingRadians(), e.getDistance()));

        if (target != null) {
            chooseTarget();
        } else {
            RobotInfo targetInfo = (RobotInfo) robots.get(target);
            if (!targetInfo.isAlive) {
                chooseTarget();
            }
        }
    }

    public void onMessageReceived(MessageEvent e) {
        RobotInfo droid = (RobotInfo) robots.get("SteinbeißerDroid");
        if (e.getMessage() == "I'm dead!") {
            soloMode = true;
            droid.setIsAlive(false);
        }
        if (e.getMessage() instanceof Point2D) {
            droid.setLocation((Point2D) e.getMessage());
        }
    }

    private String chooseTarget() {
        String chosen = null;
        //Code goes here --
        return chosen;
    }

    private void doRadar() { //zunächst nur spinning radar; auch lock bei nur einem Gegner muss noch eingebaut werden
        //http://robowiki.net/wiki/One_on_One_Radar#Spinning_radar
        if (!this.soloMode){
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        } else {
            //CODE 
        }
        
    }

    private void aimAndShoot() {
        RobotInfo bot = (RobotInfo)robots.get(target);
        final double FIREPOWER = getFirepower();
        final double ROBOT_WIDTH = 16, ROBOT_HEIGHT = 16;
        // Variables prefixed with e- refer to enemy, b- refer to bullet and r- refer to robot
        final double eAbsBearing = getHeadingRadians() + bot.getBearingRad();
        final double rX = getX(), rY = getY(),
                bV = Rules.getBulletSpeed(FIREPOWER);
        final double eX = rX + bot.distance(this.getX(), this.getY()) * Math.sin(eAbsBearing),
                eY = rY + bot.distance(this.getX(), this.getY()) * Math.cos(eAbsBearing),
                eV = bot.getVelocity(),
                eHd = bot.getHeadingRad();
        // These constants make calculating the quadratic coefficients below easier
        final double A = (eX - rX) / bV;
        final double B = eV / bV * Math.sin(eHd);
        final double C = (eY - rY) / bV;
        final double D = eV / bV * Math.cos(eHd);
        // Quadratic coefficients: a*(1/t)^2 + b*(1/t) + c = 0
        final double a = A * A + C * C;
        final double b = 2 * (A * B + C * D);
        final double c = (B * B + D * D - 1);
        final double discrim = b * b - 4 * a * c;
        if (discrim >= 0) {
            // Reciprocal of quadratic formula
            final double t1 = 2 * a / (-b - Math.sqrt(discrim));
            final double t2 = 2 * a / (-b + Math.sqrt(discrim));
            final double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);
            // Assume enemy stops at walls
            final double endX = limit(
                    eX + eV * t * Math.sin(eHd),
                    ROBOT_WIDTH / 2, getBattleFieldWidth() - ROBOT_WIDTH / 2);
            final double endY = limit(
                    eY + eV * t * Math.cos(eHd),
                    ROBOT_HEIGHT / 2, getBattleFieldHeight() - ROBOT_HEIGHT / 2);
            setTurnGunRightRadians(robocode.util.Utils.normalRelativeAngle(
                    Math.atan2(endX - rX, endY - rY)
                    - getGunHeadingRadians()));
            setFire(FIREPOWER);
        }
    }
    
    private double getFirepower(){
        double power;
        double distance;
        RobotInfo bot = (RobotInfo)robots.get(target);
        distance = bot.distance(this.getX(), this.getY());
        if (distance > 300){
            power = 0.8;
        }
        else if (distance < 75){
            power = robocode.Rules.MAX_BULLET_POWER;
        } else {
            power = 1.5;    
        }
        return power;
    }
    

    private double limit(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    private void move() {//zugriff auf hashtable: http://www.java2novice.com/java-collections-and-util/hashtable/iterate/
        //fast 1 zu 1 von: http://robowiki.net/wiki/Anti-Gravity_Tutorial
        double xForce = 0, yForce = 0;
        Set keys = robots.keySet();
        for (Object key : keys) {
            RobotInfo robot = (RobotInfo) robots.get(key);
            if (!robot.isTeammate) {
                double absBearing = Utils.normalAbsoluteAngle(Math.atan2(robot.getX() - this.getX(), robot.getY() - this.getY()));
                double distance = robot.distance(getX(), getY());
                xForce -= Math.sin(absBearing) / (distance * distance);
                yForce -= Math.cos(absBearing) / (distance * distance);
            }
        }
        double angle = Math.atan2(xForce, yForce);
        if (xForce == 0 && yForce == 0) {
            // If no force, do nothing
        } else if (Math.abs(angle - getHeadingRadians()) < Math.PI / 2) {
            setTurnRightRadians(Utils.normalRelativeAngle(angle - getHeadingRadians()));
            setAhead(Double.POSITIVE_INFINITY);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(angle + Math.PI - getHeadingRadians()));
            setAhead(Double.NEGATIVE_INFINITY);
        }
    }

    public void onDeath() {
        RobotInfo leader = (RobotInfo) robots.get("SteinbeißerLeader");
        leader.setIsAlive(false);
        try {
            broadcastMessage("I'm dead!");
        } catch (IOException ex) {
            Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    //von Shiz
    private static Point2D projectPoint(Point2D startPoint, double theta, double dist) {
        return new Point2D.Double(startPoint.getX() + dist * Math.sin(theta), startPoint.getY() + dist * Math.cos(theta));
    }

}

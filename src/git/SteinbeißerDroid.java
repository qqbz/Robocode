package git;

import java.awt.Color;
import java.util.Hashtable;
import java.awt.geom.Point2D;
import robocode.*;
import robocode.util.Utils;
import robocode.MessageEvent;
import robocode.TeamRobot;
import robocode.DeathEvent;
import static robocode.util.Utils.normalRelativeAngle;
import static robocode.util.Utils.normalRelativeAngleDegrees;

public class SteinbeißerDroid extends TeamRobot implements Droid {

    public Hashtable<String, RobotInfo> robotsList = new Hashtable<>();
    RobotInfo robotInformation = new RobotInfo();
    /**
    * Der Name des Droids.
    */
    String droidName;
    /**
    * Der Name des Leaders.
    */
    String leaderName;
    /**
    * Das aktuelle Ziel.
    */
    String target;
    /**
    * Gleichbleibende Ziel für den Droid bis das Ziel stirbt.
    */
    String targetOld;
    /**
    * 
    */
    public boolean movingForward = true;
    /**
    * Boolean Variable für geändertes Verhalten wenn der Leader zerstört wurde.
    */
    boolean soloMode = false;
    
    /*
    TO DO: 
    shoot(): Trifft den Gegner noch nicht oft
    */
    @Override
    public void run() {

        RobotColors c = new RobotColors();
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.bulletColor = Color.YELLOW;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setBulletColor(c.bulletColor);

        this.droidName = getName();
        
        do {
            if(targetOld == null){
                targetOld = target;
            }
            else if(robotsList.get(targetOld) == null) {
                 targetOld = null;   
            }
            if ((soloMode == true)) {
                soloMode();

            }
            if (targetOld != null && robotsList.get(targetOld) != null) {
                goTo(robotsList.get(targetOld));
                shoot();
                System.out.println(" got target");
            }
           
            execute();
       
        } while (true);

    }

    /**
    * Bekommt die Liste der Gegner vom Leader
    * Bekommt das momentane Ziel mitgeteil
    *
    * @param e MessageEvent
    */
    @Override
    @SuppressWarnings("empty-statement")
    public void onMessageReceived(MessageEvent e) {
       
        if (e.getMessage() instanceof String) {
            System.out.println("i received the target: "+e.getMessage());
        } else {
            this.robotsList = (Hashtable<String, RobotInfo>) e.getMessage();
            this.leaderName = e.getSender();
            this.robotInformation = robotsList.get(leaderName);
            this.target = robotInformation.getTARGET();
        }
        
    }
//schussrichtung zu ungenau muss noch besser werden
    /**
    * Zielt und schießt
    */
    public void shoot() {
 
        double firePower = 0;
        double dx = 0;
        double dy = 0;
//        double absoluteBearing = 0;
        if(targetOld != null && robotsList.get(targetOld) != null){
        dx = robotsList.get(targetOld).getX() - this.getX();
        dy = robotsList.get(targetOld).getY() - this.getY();
        }
        double distance = Math.hypot(dx, dy);
        double theta = Math.toDegrees(Math.atan2(dx, dy));

        turnGunRight(normalRelativeAngleDegrees(theta - getGunHeading()));

        System.out.println(" Target " + targetOld + " Distance " + distance);
        
//        absoluteBearing = getHeadingRadians() + 
//            robotsList.get(target).getBearingRad();
//        setTurnGunRight(robocode.util.Utils.normalRelativeAngle
//            (absoluteBearing - getGunHeadingRadians()));        

        if (distance > 700) {
            firePower = 0.5;
        } else if (distance <= 200) {
            firePower = robocode.Rules.MAX_BULLET_POWER;
        } else {
            firePower = 4 - (distance / 200);
        }
        System.out.println(" Fire with "+firePower);
        fire(firePower);

    }    
    
    /**
    * Lässt den Bot zu den angegebenen Koordinaten fahren. Kleine Abweichungen sind möglich.
    * @param x x-Koordinate des Zielpunktes
    * @param y y-Koordinate des Zielpunktes
    *           //http://www.jasonsjava.com/?cat=2
    */
    private void goTo(Point2D destination) {
        Point2D location = new Point2D.Double(getX(), getY());
        double distance = location.distance(destination);
        double angle = Utils.normalRelativeAngleDegrees(Math.toDegrees(Math.atan2(destination.getX() - location.getX(), destination.getY() - location.getY())) - getHeading());
        if (Math.abs(angle) > 90) {
            distance *= -1;  
            if (angle > 0) {
                angle -= 180;
            } else {
                angle += 180;
            }
        }

        turnRight(angle); 
        setAhead(distance);
        execute();
    }

    /**
    * Überprüfen ob eine Runde wegen zu langen Berechnungen übersprungen wurde.
    *
    * @param e SkippedTurnEvent
    */
    @Override
    public void onSkippedTurn(SkippedTurnEvent e) {
        System.out.println("Round " + e.getSkippedTurn() + " was skipped!");
    }

    /**
    * Dreht bei Treffen des Leaders sich um
    * Drehet sich durch reverseDirection()
    * 
    * else: Gegner hitten auf ihn zielen //robowiki.net/wiki/Fusion
    *
    * @param e SkippedTurnEvent
    */
    @Override
    public void onHitRobot(HitRobotEvent e) {
        if (isTeammate(e.getName())) {
            reverseDirection();
        }
        else{
            double absB = getHeadingRadians() + e.getBearingRadians();
            setTurnGunRightRadians(normalRelativeAngle(absB-getGunHeading()));
            //setAhead(10);
            System.out.println("hit dat guy");
            shoot();
        }
    }

    /**
    * Dreht sich wenn er gegen die Wand fährt 
    * Drehet sich durch reverseDirection()
    *
    * @param e SkippedTurnEvent
    */    
    @Override
    public void onHitWall(HitWallEvent e) {
        out.println("I hit the wall " + e.getBearing());
        reverseDirection();
    }
    
    /**
    * Fragt ob Leader gestorben ist, wenn ja wechselt in soloMode
    * 
    * @param e RobotDeathEvent
    */
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        String deadBot = e.getName();
        //target=null;
        if (isTeammate(deadBot)) {
            System.out.println(e.getName() + " is Dead!");
            soloMode = true;
        }
        if(deadBot.equals(targetOld)){
            targetOld = null;
            System.out.println("alte ziel tot");
        }
       
    }
    
    
    /**
    * Der soloMode des Droiden
    * 
    * Crazy - a sample robot by Mathew Nelson.
    */
    public void soloMode() {
        setAhead(40000);
        movingForward = true;
        setTurnRight(90);
        waitFor(new TurnCompleteCondition(this));
        setTurnLeft(180);
        waitFor(new TurnCompleteCondition(this));
        setTurnRight(180);
        waitFor(new TurnCompleteCondition(this));

    }
    
    /**
    * Gibt die Richtung und den Winkel des wegdrehens an
    * Fährt dann 100 in diese Richtung
    * 
    */
    public void reverseDirection() {
        if (movingForward) {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setBack(100);
            movingForward = false;
        } else {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setAhead(100);
            movingForward = true;
        }
    }
}

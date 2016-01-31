package git;

import java.awt.Color;
import java.util.Hashtable;
import java.awt.geom.Point2D;
import robocode.*;
import robocode.util.Utils;
import robocode.MessageEvent;
import robocode.TeamRobot;
import static robocode.util.Utils.normalRelativeAngleDegrees;

/**
 * SteinbeißerDroid ohne Radar
 *
 * @author Alex Dercho, Beiträge von Lukas Becker
 */
public class SteinbeißerDroid extends TeamRobot implements Droid {

    /**
     * Liste aller gescannter Bots
     */
    public Hashtable<String, RobotInfo> robotsList = new Hashtable<>();

    /**
     * Infodatei für gescannten Bot
     */
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
     * Das aktuelle Ziel des Leaders.
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

    @Override
    public void run() {

        setBodyColor(Color.black);
        setGunColor(Color.green);
        setBulletColor(Color.green);

        this.droidName = getName();

        double hoehe = this.getBattleFieldHeight();
        double weite = this.getBattleFieldWidth();

        do {
            if (targetOld == null) { //                         solange der Gegner nicht zerstört, Ziel beibehalten
                targetOld = target;
            } else if (robotsList.get(targetOld) == null) { //  wenn der Gegner vernichtet wurde(targetOld), lösche aus der Liste
                targetOld = null;
            }
            if (soloMode == true) { //                          Leader tot, geh in den Crazy-Modus
                crazyMode();
            }
            if (targetOld != null && robotsList.get(targetOld) != null && hoehe < 1200 && weite < 1200) {// Target vom Leader beim größerem Feld
                goTo(robotsList.get(targetOld));
                shoot(false);
                //               System.out.println(" got target");
            } else if (target != null) { //                     wenn Ziel vorhanden, fahre auf das Ziel und Schieße
                goTo(robotsList.get(target));
                shoot(true);
            } else if (target == null) {  //                    wenn noch kein Ziel, fahre im Crazy-Modus
                crazyMode();
            }

            execute();

        } while (true);

    }

    /**
     * Bekommt die Liste der Gegner vom Leader Bekommt das momentane Ziel
     * mitgeteil
     *
     * @param e MessageEvent
     */
    @Override
    @SuppressWarnings("empty-statement")
    public void onMessageReceived(MessageEvent e) {

        if (e.getMessage() instanceof String) {
            //           System.out.println("Momentanes Target des Leaders: "+e.getMessage());
            this.target = (String) e.getMessage();
        } else {
//           System.out.println("Hashtable wurde aktualiesiert");
            this.robotsList = (Hashtable<String, RobotInfo>) e.getMessage();
            this.leaderName = e.getSender();
            this.robotInformation = robotsList.get(leaderName);
            //this.target = robotInformation.getTARGET();
        }

    }

    /**
     * Zielt und schießt Die Feuerkraft ist abhängig von der Entfernung zum Ziel
     *
     * @param zuGross
     */
    public void shoot(boolean zuGross) {

        double firePower = 0;
        double dx = 0;
        double dy = 0;
//        double absoluteBearing = 0;
        if (targetOld != null && robotsList.get(targetOld) != null && !zuGross) {
            dx = robotsList.get(targetOld).getX() - this.getX();
            dy = robotsList.get(targetOld).getY() - this.getY();
        } else if (target != null && zuGross) {
            dx = robotsList.get(target).getX() - this.getX();
            dy = robotsList.get(target).getY() - this.getY();
        }
        double distance = Math.hypot(dx, dy);
        double theta = Math.toDegrees(Math.atan2(dx, dy));
        turnGunRight(normalRelativeAngleDegrees(theta - getGunHeading()));
//        System.out.println(" Target " + targetOld + " Distance " + distance);   
        if (distance <= 100) {          //distanzabhängiges Schießen
            firePower = robocode.Rules.MAX_BULLET_POWER;
            fire(firePower);
        } else if (distance <= 300) {
            firePower = 4.5 - (3 * (distance / 200));
            fire(firePower);
        }
//        System.out.println("Das Ziel ist: " +target);

        System.out.println("Entfernung: " + distance);
        System.out.println("Schussstärke: " + firePower);

    }

    /**
     * Lässt den Bot zu den angegebenen Koordinaten fahren. Kleine Abweichungen
     * sind möglich.
     *
     * @param x x-Koordinate des Zielpunktes
     * @param y y-Koordinate des Zielpunktes //http://www.jasonsjava.com/?cat=2
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
     * Dreht bei Treffen des Leaders sich um Drehet sich durch
     * reverseDirection()
     *
     * else: Gegner hitten auf ihn zielen //robowiki.net/wiki/Fusion
     *
     *
     * Meldet wenn der Droid den Teammate gerammt hat. (Zur Überprüfung eines
     * Testkriteriums)
     *
     * @param e SkippedTurnEvent
     */
    @Override
    public void onHitRobot(HitRobotEvent e) {
        
        if (isTeammate(e.getName())) {
            setBack(200);
        }
//                         System.out.println("Teammate gerammt!");
        else {
            double absB = getHeading() + e.getBearing();
            setTurnGunRight(normalizeBearing(absB - getGunHeading()));
//                        System.out.println("hit dat guy");
            fire(3);
        }
    }

    /**
     * Normalisiert Bearing, um die Bewegung effizienter zu machen.
     *
     * Aus http://mark.random-article.com/weber/java/robocode/lesson4.html
     */
    double normalizeBearing(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Dreht sich wenn er gegen die Wand fährt Drehet sich durch
     * reverseDirection()
     *
     * Meldet wenn der Droid die Wand gerammt hat. (Zur Überprüfung eines
     * Testkriteriums)
     *
     * @param e SkippedTurnEvent
     */
    @Override
    public void onHitWall(HitWallEvent e) {
        //       System.out.println("Wand gerammt !");
        reverseDirection();
    }

    /**
     * Fragt ob Leader gestorben ist, wenn ja wechselt in soloMode Fragt ob das
     * momentane Ziel des Droiden gestroben ist
     *
     * @param e RobotDeathEvent
     */
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        String deadBot = e.getName();
        if (isTeammate(deadBot)) {
            //           System.out.println(e.getName() + " is Dead!");
            soloMode = true;
            target = null;  //verhindert sinnloses schießen im soloMode
            targetOld = null;
        }
        if (deadBot.equals(targetOld)) {
            targetOld = null;
            //           System.out.println("alte ziel tot");
        }

    }

    /**
     * Meldet wenn der Droid vom Leader getroffen wurde. (Zur Überprüfung eines
     * Testkriteriums)
     *
     * @param e SkippedTurnEvent
     */
    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        if (isTeammate(e.getName())) {
            //          System.out.println(e.getName() + " hat mich Abgeschossen");
        }
    }

    /**
     * Meldet wenn der Droid einen Gegner trifft. (Zur Überprüfung eines
     * Testkriteriums)
     *
     * @param e SkippedTurnEvent
     */
    @Override
    public void onBulletHit(BulletHitEvent e) {
        //     System.out.println("Gegner: " + e.getName() + " getroffen");
    }

    /**
     * Der soloMode des Droiden.
     *
     * Crazy - a sample robot by Mathew Nelson (Angepasst)
     *
     */
    public void crazyMode() {
        setAhead(40000);
        movingForward = true;
        setTurnRight(90);
        if (target == null) {                          // wenn noch kein Ziel soll warten bis der Befehl ausgeführt wird, ansonsten raus aus Crazy-Mode
            waitFor(new TurnCompleteCondition(this)); // wartet bis die Anweisung ausgeführt wurde
            setTurnLeft(180);
            if (target == null) {
                waitFor(new TurnCompleteCondition(this));
                setTurnRight(180);
                if (target == null) {
                    waitFor(new TurnCompleteCondition(this));
                }
            }
        }

    }

    /**
     * Gibt die Richtung und den Winkel des wegdrehens an Fährt dann 100 in
     * diese Richtung
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


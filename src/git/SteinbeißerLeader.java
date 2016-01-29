package git;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.LinkedHashMap;
import robocode.HitWallEvent;
import robocode.ScannedRobotEvent;
import robocode.TeamRobot;
import robocode.MessageEvent;
import robocode.RobotDeathEvent;
import robocode.SkippedTurnEvent;
import robocode.util.Utils;

/**
 * SteinbeißerLeader mit Radar
 *
 * @author Simon Fella, Beiträge von Lukas Becker
 */
public class SteinbeißerLeader extends TeamRobot {

    /**
     * Die Liste aller Roboter mit deren Daten.
     */
    private Hashtable<String, RobotInfo> robots = new Hashtable<>();
    /**
     * Das aktuelle Ziel.
     */
    private String target;
    /**
     * Boolean ob der Leader alleine kämpft. Stellt die Kommunikation ein, wenn
     * es keinen Empfänger gibt.
     */
    private boolean soloMode = false;
    /**
     * Boolean ob nur noch ein einzelner Gegner übrig ist.
     */
    private boolean singleEnemy = false;
    /**
     * Boolean ob kein Gegner in Reichweite des Radars ist. Wird bei größeren
     * Arenen benötigt, da sich dort Gegner außerhalb der Reichweite des Radar
     * aufhalten können.
     */
    private boolean noEnemyInScanRadius = true;
    /**
     * Zähler wie lange schon kein Gegner gescant wurde. Wird verwendet um in
     * die Mitte der Arena zu fahren, wenn länger kein Gegner gesehen wird.
     */
    private int counterNoEnemyFound = 0;
    /**
     * Zähler für Anzahl der gestorbenen Verbündeten. Wird verwendet um die
     * Kommunikation einzustellen, wenn es eh keinen Empfänger gibt.
     */
    private int deadAllies = 0;
    //private long timeOld = -50; //Entfernt, da ein schneller Gegnerwechsel nichtmehr gegen die Strategie geht.
    //private int forward = 1; //Entfernt, da die Bewegung die Siegesrate wesentlich verringerte.

    //Variablen die direkt von Quellen übernommen wurden
    private double scanDir;
    private Object sought;
    private LinkedHashMap<String, Double> enemyHashMap;
    private double oldEnemyHeading;

    //Konstancen
    private double BATTLEFIELDHEIGHT, BATTLEFIELDWIDTH;
    private final int RADIUSPOINTGENERATION = 100;
    private final double SAFEWALLDISTANCE = 40.;
    private final int MAXDISTANCETOENEMY = 1000;
    private final double MINRADIUSGENERATION = 50.;
    private final double MAXRADIUSGENERATION = 300.;

    //Variablen zum Debuggen
    private double aSchusslinie, bSchusslinie;
    private long stoppingTime;
    private Point2D m;
    private Point2D generatedPoints[];

    /**
     * Hauptmethode, die alle anderen Methoden startet.
     */
    @Override
    public void run() {
        BATTLEFIELDHEIGHT = getBattleFieldHeight(); //Initialisierung der beiden Konstanten
        BATTLEFIELDWIDTH = getBattleFieldWidth();

        singleEnemy = false;

        setBodyColor(Color.black); //setzt die Farben des Roboters
        setGunColor(Color.green);
        setRadarColor(Color.green);
        setScanColor(Color.green);
        setBulletColor(Color.green);

        setAdjustRadarForGunTurn(true); //Parametrisierung des Radars
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForRobotTurn(true);
        scanDir = 1;
        enemyHashMap = new LinkedHashMap<>(5, 2, true);
        setTurnRadarRightRadians(Double.POSITIVE_INFINITY);

        RobotInfo leader = new RobotInfo(); //fügt sich selbst der Liste hinzu
        robots.put(this.getName(), leader);

        //System.out.println("orangener Kreis: \tmaximaler Abstand von 1000 Einheiten");
        //System.out.println("rotes Rechteck: \tZielpunkt der momentanen Bewegung");
        //System.out.println("weiße Linie: \t\tmomentane Schusslinie");
        stoppingTime = 0; //zum Prüfen eines Testfalles

        do {
            leader = (RobotInfo) robots.get(this.getName()); //Update der eigenen Werte
            leader.setLocation(this.getX(), this.getY());
            leader.setEnergy(this.getEnergy());
            leader.setHeadingRad(this.getHeading());
            leader.setVelocity(this.getVelocity());
            leader.setHeadingRad(this.getHeadingRadians());

            counterNoEnemyFound++; //Prüfung ob schon zulange kein Gegner gesehen wurde.
            if (counterNoEnemyFound > 30) {
                noEnemyInScanRadius = true;
            }

            doRadar();
            chooseTarget();
            aimAndShoot();
            //ausgabeRoboterliste();
            //checkStandingTime();

            if (!soloMode) { //sendet Roboter-Liste und das momentane Ziel an den Droid
                try {
                    broadcastMessage(robots);
                    if (target != null) {
                        broadcastMessage(target);
                    }
                } catch (IOException ex) {
                }

            }

            move();
            execute();

        } while (true);

    }

    /**
     * Malt in die Arena zum graphisches Debuggen.
     *
     * @param g Graphics2D
     */
    @Override
    public void onPaint(Graphics2D g) {
        try {
            g.setColor(new Color(0xff, 0xff, 0xff, 0x80));
            g.drawLine((int) getX(), (int) getY(), (int) aSchusslinie, (int) bSchusslinie); //Malt eine Linie in dem Winkel in dem momentan gezielt wird.
            g.drawOval((int) getX() - MAXDISTANCETOENEMY, (int) getY() - MAXDISTANCETOENEMY, 2 * MAXDISTANCETOENEMY, 2 * MAXDISTANCETOENEMY);
            //Malt einen Kreis mit dem Radius MAXDISTANCETOENEMY. Zeigt wie weit ein Gegner maximal entfernt sein darf, dass er noch als Ziel zugelassen ist.
            g.drawRect((int) SAFEWALLDISTANCE, (int) SAFEWALLDISTANCE, (int) (BATTLEFIELDWIDTH - 2 * SAFEWALLDISTANCE), (int) (BATTLEFIELDHEIGHT - 2 * SAFEWALLDISTANCE));
            //Malt ein Rechteck in dem jeder generierte Bewegungspunkt liegen muss um einen Zusammenstoß mit der Wand zu vermeiden.
            g.drawOval((int) getX() - RADIUSPOINTGENERATION, (int) getY() - RADIUSPOINTGENERATION, 2 * RADIUSPOINTGENERATION, 2 * RADIUSPOINTGENERATION);
            //Malt einen Kreis in dem ein Gegner bei der Generierung der Bewegungspunkte berücksichtigt wird.
            g.drawOval((int) (getX() - MINRADIUSGENERATION), (int) (getY() - MINRADIUSGENERATION), (int) (2 * MINRADIUSGENERATION), (int) (2 * MINRADIUSGENERATION));
            //Malt einen Kreis außerhalb dem die generierten Bewegungspunkte liegen müssen.
            g.drawOval((int) (getX() - MAXRADIUSGENERATION), (int) (getY() - MAXRADIUSGENERATION), (int) (2 * MAXRADIUSGENERATION), (int) (2 * MAXRADIUSGENERATION));
            //Malt einen Kreis innerhalb dem die generirten Bewegungspunkte liegen müssen.
            g.setColor(new Color(0xff, 0xcf, 0x00, 0x80));
            try {
                for (Point2D generatedPoint : generatedPoints) {
                    g.fillRect((int) (generatedPoint.getX() - 10), (int) (generatedPoint.getY() - 10), 20, 20);
                }
            } catch (Exception e) {
            }   //Malt alle genrierten Bewegungspunkte.
            g.setColor(new Color(0xff, 0x00, 0x00, 0x80));
            g.fillRect((int) m.getX() - 10, (int) m.getY() - 10, 20, 20);
            //Malt den gewählten Bewegungspunkt in besonderer Farbe.
            g.drawLine((int) getX(), (int) getY(), (int) m.getX(), (int) m.getY());
            //Mal eine Linie von der Position des Leader zum gewählten Bewegungspunkt.
        } catch (Exception e) {
        }
    }

    /**
     * Updated die Position des Droid wenn er sie sendet.
     *
     * @param e MessageEvent
     */
    @Override
    public void onMessageReceived(MessageEvent e) {
        RobotInfo droid = (RobotInfo) robots.get(e.getSender());
        if (e.getMessage() instanceof Point2D) {
            droid.setLocation((Point2D) e.getMessage());
        }
    }

    /**
     * Radar-Steuerung. SpinningRadar wenn mehrere Gegner vorhanden sind,
     * Radar-Lock wenn nur ein Gegner vorhanden ist.
     */
    private void doRadar() {
        if (singleEnemy && !noEnemyInScanRadius) {
            //System.out.println("s");
            scan();
        } else {
            //System.out.println("m");
            setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
            scan();
        }

        //Berechnet die Anzahl der verbleibenden Gegner
        String[] teammates = getTeammates();
        int numberOfEnemiesAlive;
        if (teammates != null) {
            numberOfEnemiesAlive = getOthers() - teammates.length + deadAllies;
        } else if (teammates == null && !soloMode) {
            numberOfEnemiesAlive = getOthers();
            soloMode = true;
        } else {
            numberOfEnemiesAlive = getOthers();
        }

        if (!singleEnemy && numberOfEnemiesAlive == 1) {
            singleEnemy = true;
            //System.out.println("single enemy");
        }
    }

    /**
     * Zielt und schießt. Nutzt einen Circular Tergeting-Algorithmus. Quelle:
     * http://robowiki.net/wiki/Circular_Targeting
     */
    private void aimAndShoot() {
        if (target != null) {
            RobotInfo e = robots.get(target);
            if (e.distance(getX(), getY()) < MAXDISTANCETOENEMY) {
                double bulletPower = getFirepower();
                double myX = getX();
                double myY = getY();
                double absoluteBearing = getHeadingRadians() + e.getBearingRad();
                double enemyX = getX() + e.distance(getX(), getY()) * Math.sin(absoluteBearing);
                double enemyY = getY() + e.distance(getX(), getY()) * Math.cos(absoluteBearing);
                double enemyHeading = e.getHeadingRad();
                double enemyHeadingChange = enemyHeading - oldEnemyHeading;
                double enemyVelocity = e.getVelocity();
                oldEnemyHeading = enemyHeading;
                double deltaTime = 0;
                double predictedX = enemyX, predictedY = enemyY;
                while ((++deltaTime) * (20.0 - 3.0 * bulletPower) < Point2D.Double.distance(myX, myY, predictedX, predictedY)) {
                    predictedX += Math.sin(enemyHeading) * enemyVelocity;
                    predictedY += Math.cos(enemyHeading) * enemyVelocity;
                    enemyHeading += enemyHeadingChange;
                    if (predictedX < 18.0 || predictedY < 18.0 || predictedX > BATTLEFIELDWIDTH - 18.0 || predictedY > BATTLEFIELDHEIGHT - 18.0) {
                        predictedX = Math.min(Math.max(18.0, predictedX), BATTLEFIELDWIDTH - 18.0);
                        predictedY = Math.min(Math.max(18.0, predictedY), BATTLEFIELDHEIGHT - 18.0);
                        break;
                    }
                }
                double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - getX(), predictedY - getY()));
                aSchusslinie = getX() + 1500 * Math.sin(theta);
                bSchusslinie = getY() + 1500 * Math.cos(theta);
                theta = Utils.normalRelativeAngle(theta - getGunHeadingRadians());
                setTurnGunRightRadians(theta);
                //System.out.println(bulletPower);
                fire(bulletPower);
            }
        }
    }

    /**
     * Berechnet die Feuerkraft in Abhängigkeit von der Entfernung zum
     * momentanen Ziel mit einer linearen Funktion.
     *
     * @return Berechnete Feuerkraft
     */
    private double getFirepower() {
        double power;
        double distance;
        RobotInfo bot = (RobotInfo) robots.get(target);
        distance = bot.distance(this.getX(), this.getY());
        if (distance > 700) {
            power = 0.1;
        } else if (distance <= 200) {
            power = robocode.Rules.MAX_BULLET_POWER;
        } else {
            power = 4.16 - (distance * 2.9 / 500);
        }
        return power;
    }

    /**
     * Startet die Generierung und Evaluierung der möglichen Bewegungspunkte und
     * bewegt sich zum ausgewählten Zielpunkt.
     */
    private void move() {
        Point2D ziel;
        if (getDistanceRemaining() == 0 && noEnemyInScanRadius) {
            target = null;
            singleEnemy = false;
            setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
            int halbeBreite = (int) BATTLEFIELDWIDTH / 2;
            int halbeHöhe = (int) BATTLEFIELDHEIGHT / 2;
            int xAbweichung = halbeBreite - (int) getX();
            int yAbweichung = halbeHöhe - (int) getY();
            int x = halbeBreite - xAbweichung / 20;
            int y = halbeHöhe - yAbweichung / 20;
            goTo(x, y);
        } else if (!noEnemyInScanRadius) {
            ziel = evaluate(generate());
            m = ziel;
            goTo((int) ziel.getX(), (int) ziel.getY());
        }
        /*} else { //Ehermals: Bewegungn wenn der Leader alleine ist. Entfernt, da dies die Siegesrate drastisch reduzierte.
         if (getDistanceRemaining() == 0) { 
         int dist = (int)(Math.random() * 150) + 50;
         forward = -forward; 
         setAhead(dist * forward); 
         }
         RobotInfo e = (RobotInfo)robots.get(target);
         setTurnRightRadians(e.getBearingRad() + Math.PI/2 - 0.5236 * forward * (e.distance(getX(), getY()) > 200 ? 1 : -1));
         }*/
    }

    /**
     * Lässt den Bot zu den angegebenen Koordinaten fahren. Kleine Abweichungen
     * sind möglich. Quelle: http://robowiki.net/wiki/GoTo
     *
     * @param x x-Koordinate des Zielpunktes
     * @param y y-Koordinate des Zielpunktes
     */
    private void goTo(int x, int y) {
        double alpha;
        setTurnRightRadians(Math.tan(alpha = Math.atan2(x -= (int) getX(), y -= (int) getY()) - getHeadingRadians()));
        setAhead(Math.hypot(x, y) * Math.cos(alpha));
    }

    /**
     * Wählt den sichersten Punkt in der übergebenen ArrayList aus. Quelle für
     * die Risiko-Berechnung:
     * http://robowiki.net/wiki/Melee_Strategy/UnderstandingHawkOnFire
     *
     * @param pointarray ArrayListe aller Punkte die evaluiert werden sollen
     * @return den sichersten Punkt der Liste
     */
    private Point2D evaluate(ArrayList pointList) {
        double minrisk = Double.POSITIVE_INFINITY;
        double thisrisk;
        double energyratio;
        double perpendicularity;
        double distanceSq;
        Point2D bestPoint = new Point2D.Double();
        //Berechnet das Risiko für jeden einzelnen Punkt.
        for (Object point1 : pointList) {
            Point2D p = (Point2D) point1;
            if (point1 != null) {
                //Berechnet das Risiko jedes Gegners für den Punkt.
                thisrisk = 0;
                for (String key : robots.keySet()) {
                    RobotInfo r = robots.get(key);
                    energyratio = r.getEnergy() / this.getEnergy();
                    perpendicularity = Math.abs(Math.cos(Math.atan2(p.getY() - r.getY(), p.getX() - r.getX()) - Math.atan2(p.getY() - this.getY(), p.getX() - this.getX())));
                    distanceSq = r.distanceSq(p);
                    thisrisk += energyratio * (1 + perpendicularity) / distanceSq;
                }
                //Wenn das Risiko des Punktes niedriger ist als das niedrigste Risiko bisher wird diser Punkt der neue beste Punkt.
                if (thisrisk < minrisk) {
                    minrisk = thisrisk;
                    bestPoint = p;
                }
            }
        }
        //Gibt den Punkt mit dem niedrigsten Risiko zurück.
        return bestPoint;
    }

    /**
     * Genertiert bis zu 200 Punkte im Bereich von MINRADIUSGENERATION bis
     * MAXRADIUSGENERATION Einheiten um den Bot die um SAFEWALLDISTANCE von der
     * Wand entfernt sind.
     *
     * @return die genertierten Punkte
     */
    public ArrayList generate() {
        ArrayList points = new ArrayList(1);
        double theta;
        double dist;
        boolean pointBehindEnemy;
        if (target != null) {
            RobotInfo targetInfo = (RobotInfo) robots.get(target);
            for (int i = 0; i < 200; i++) {
                //Generierung eines zufälligen Punktes um den Leader.
                theta = Math.random() * Math.PI * 2.0;
                dist = Math.random() * (MAXRADIUSGENERATION - MINRADIUSGENERATION) + MINRADIUSGENERATION;
                Point2D p = projectPoint(new Point2D.Double(this.getX(), this.getY()), theta, dist);
                //Prüfung ob der Punkt hinter einem nahen Roboter liegt, wenn ja kommt er nicht als Zielpunkt in Frage, damit der Leader nicht in den Roboter hineinfährt.
                pointBehindEnemy = false;
                for (String key : robots.keySet()) {
                    if (!this.getName().equals(key)) {
                        RobotInfo r = robots.get(key);
                        double robotAbsolutBearing = getHeadingRadians() + r.getBearingRad();
                        double d = r.distance(getX(), getY());
                        if (d < RADIUSPOINTGENERATION && !(theta < robotAbsolutBearing - Math.PI / 2 || theta > robotAbsolutBearing + Math.PI / 2)) {
                            pointBehindEnemy = true;
                        }
                    }
                }
                //Prüfung ob der Punkt nicht zu nahe an der Wand liegt, wenn ja kommt er als Zielpunkt nicht in Frage, damit der Leader nicht gegen die Wand fährt.
                //Zudem darf der Punkt nicht mehr als 1000 Einheiten vom Target entfernt sein, damit dieser nicht aus der Reichweite unseres Radars gelangt.
                if (p.getX() > SAFEWALLDISTANCE && p.getX() < BATTLEFIELDWIDTH - 2 * SAFEWALLDISTANCE && p.getY() > SAFEWALLDISTANCE
                        && p.getY() < BATTLEFIELDHEIGHT - 2 * SAFEWALLDISTANCE && p.distance(targetInfo.getX(), targetInfo.getY()) < MAXDISTANCETOENEMY && !pointBehindEnemy) {
                    points.add(p);
                }
            }
        } else {
            //Hat der Leader noch kein Target, so ist die RoboterListe wahrscheinlich noch nicht vollständig. Daher vernachlässigt der Leader hier die Prüfung ob der Punkt hinter einem Roboter liegt.
            //Kann nur wenige Ticks am Anfang eines Battles eintreten.
            for (int i = 0; i < 200; i++) {
                theta = Math.random() * Math.PI * 2.0;
                dist = Math.random() * 250.0 + 50.0;
                Point2D p = projectPoint(new Point2D.Double(this.getX(), this.getY()), theta, dist);
                if (p.getX() > SAFEWALLDISTANCE && p.getX() < BATTLEFIELDWIDTH - 2 * SAFEWALLDISTANCE && p.getY() > SAFEWALLDISTANCE
                        && p.getY() < BATTLEFIELDHEIGHT - 2 * SAFEWALLDISTANCE) {
                    points.add(p);
                }
            }
        }
        //Fügt die erzeugten Punkte dem Array generatedPoints zum Debuggen hinzu.
        generatedPoints = new Point2D[points.size()];
        for (int i = 0; i < points.size(); i++) {
            generatedPoints[i] = (Point2D) points.get(i);
        }

        return points;
    }

    /**
     * Trägt die Daten des gescannten Roboters in die Liste ein bzw. updated
     * diese. Quelle für Radar-Lock: http://robowiki.net/wiki/One_on_One_Radar
     * Quelle für mehrere Gegner: http://robowiki.net/wiki/Melee_Radar
     *
     * @param e ScannedRobotEvent
     */
    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        if (!isTeammate(e.getName())) { //Resettet den Zähler und die Boolean die anzeigen ob schon länger kein Gegner gesichtet wurde.
            counterNoEnemyFound = 0;
            noEnemyInScanRadius = false;
        }
        if (this.singleEnemy && !isTeammate(e.getName())) {
            //Lässt das Radar langsam auf das Ziel fokusieren.
            double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
            setTurnRadarRightRadians(1.9 * Utils.normalRelativeAngle(radarTurn));
        } else {
            //Dreht das Radar bis alle Roboter einmal gescannt wurden. Dreht danach die Richtung um und beginnt von vorne.
            String name = e.getName();
            LinkedHashMap<String, Double> ehm = enemyHashMap;
            ehm.put(name, getHeadingRadians() + e.getBearingRadians());
            if ((name == sought || sought == null) && ehm.size() == getOthers()) {
                scanDir = Utils.normalRelativeAngle(ehm.values().iterator().next() - getRadarHeadingRadians());
                sought = ehm.keySet().iterator().next();
            }
        }
        //Updated die Werte des gescanten Roboters.
        //Vorgehen angelehnt an den Roboter "Shiz"
        String scanName = e.getName();
        if (!robots.containsKey(scanName)) {
            RobotInfo robot = new RobotInfo();
            // robot.NAME = scanName;
            robots.put(scanName, robot);
        }
        RobotInfo robot = (RobotInfo) robots.get(scanName);
        robot.setEnergy(e.getEnergy());
        robot.setVelocity(e.getVelocity());
        robot.setHeadingRad(e.getHeadingRadians());
        robot.setBearingRad(e.getBearingRadians());
        robot.setLocation(projectPoint(new Point2D.Double(this.getX(), this.getY()), getHeadingRadians() + e.getBearingRadians(), e.getDistance()));
    }

    /**
     * Wählt ein neues Ziel aus. Damit ein Gegner das neue Ziel werden kann muss
     * er dem Leader um 100 Einheiten näher kommen als das momentane Ziel
     * entfernt ist.
     */
    private void chooseTarget() {
        //wechselt Ziel wenn ein Gegner dem Leader um 100 Einheiten näher ist als das momentan Ziel entfern ist

        String closest = null;
        Point2D myPosition = new Point2D.Double(this.getY(), this.getX());
        double minDistance = Double.POSITIVE_INFINITY;
        double botDistance;
        double targetDistance;
        //long timeWait = 50; //Ticks die gewartet werden bevor ein Targetwechsel zugelassen wird - entfernt, da für die Strategie nichtmehr notwendig
        //long timeNow;
        RobotInfo bot;
        //Errechnet die Entfernung zum momentanen Ziel, wenn es denn eins gibt
        if (target != null) {
            bot = (RobotInfo) robots.get(target);
            targetDistance = bot.distance(myPosition);
            if (targetDistance > MAXDISTANCETOENEMY) {
                target = null;
            }
        } else {
            targetDistance = Double.POSITIVE_INFINITY;
        }
        //Errechnet die Entfernung zum nähesten Gegner
        for (String key : robots.keySet()) {
            bot = (RobotInfo) robots.get(key);
            if (!isTeammate((String) key)) {
                botDistance = bot.distance(myPosition);
                if (botDistance < minDistance) {
                    minDistance = botDistance;
                    closest = (String) key;
                }
            }
        }
        //timeNow = this.getTime();
        //System.out.println("old = " + timeOld + " now = " + timeNow);

        //wenn ein Gegner um 100 Einheiten näher zum Leader ist als das Ziel wird er das neue Ziel
        //die Zeitvariable verhindert ein zu schnelles Wechseln
        //if (minDistance < targetDistance - 100. && timeNow > timeOld + timeWait) {
        if (minDistance < targetDistance - 100.) {
            //timeOld = timeNow;
            this.target = closest;
        }
        //System.out.println(this.target);
    }

    /**
     * Überprüfen ob eine Runde wegen zu langen Berechnungen übersprungen wurde.
     * Zum Debuggen.
     *
     * @param e SkippedTurnEvent
     */
//    @Override
//    public void onSkippedTurn(SkippedTurnEvent e) {
//        System.out.println("Round " + e.getSkippedTurn() + " was skipped!");
//    }
    /**
     * Löscht den Eintrag des getöteten Roboters aus der Liste. Ruft
     * chooseTarget() auf wenn der getöteten Roboters das momentane Ziel war.
     * Wechselt in den soloMode wenn der getöteten Roboters das Teammitglied
     * war.
     *
     * @param e RobotDeathEvent
     */
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        String deadBot = e.getName();
        robots.remove(deadBot);
        enemyHashMap.remove(deadBot);

        if (deadBot.equals(target)) { //Lässt ein neues Ziel suchen, wenn der getötete Roboter das alte Ziel war
            target = null;
            chooseTarget();
        }
        sought = null;
        String[] teammates = getTeammates();
        if (isTeammate(deadBot)) {
            deadAllies++;
            if (teammates != null && teammates.length == deadAllies) {
                soloMode = true;
                //System.out.println("soloMode");
            }
        }
    }

    /**
     * Berechnet einen Punkt aus einem Startpunkt, dem Winkel und dem Abstand.
     * Quelle: Roboter "Shiz"
     *
     * @param startPoint Startpunkt von dem gemessen wurde
     * @param theta Winkel zum Messpunkt
     * @param dist Entfernung zum Messpunkt
     * @return Berechneter Punkt
     */
    private static Point2D projectPoint(Point2D startPoint, double theta, double dist) {
        return new Point2D.Double(startPoint.getX() + dist * Math.sin(theta), startPoint.getY() + dist * Math.cos(theta));
    }

    /**
     * Gibt die wichtigsten Daten des robots-Hashtables aus. (Zur Überprüfung
     * eines Testkriteriums)
     */
//    private void ausgabeRoboterliste() {
//        try {
//            int i = 0;
//            int highestTabCounter = 0;
//            for (String key : robots.keySet()) {
//                int tabCounter = key.length() / 4;
//                if (tabCounter > highestTabCounter) {
//                    highestTabCounter = tabCounter;
//                }
//            }
//            System.out.print("Nummer\tName");
//            for (int j = 0; j < highestTabCounter; j++) {
//                System.out.print("\t");
//            }
//            System.out.println("x-Koord.\ty-Koord.\tGeschw.\t\tEnergie");
//            System.out.println("---------------------------------------------------------------------------");
//            for (String key : robots.keySet()) {
//                RobotInfo info = (RobotInfo) robots.get(key);
//                i++;
//                System.out.print(i + "\t\t" + key);
//                for (int j = 0; j < highestTabCounter + 1 - key.length() / 4; j++) {
//                    System.out.print("\t");
//                }
//                System.out.printf("%.2f\t\t%.2f\t\t%.2f\t\t%.2fn", info.x, info.y, info.getVelocity(), info.getEnergy());
//                System.out.println("");
//            }
//            System.out.println("");
//            System.out.println("");
//        } catch (Exception e) {
//        }
//    }
    /**
     * Meldet wenn der Leader die Wand gerammt hat. (Zur Überprüfung eines
     * Testkriteriums)
     *
     * @param event
     */
//    @Override
//    public void onHitWall(HitWallEvent event) {
//        out.println("wall bei x: "+ getX()+"     y: "+getY());
//    }
    /**
     * Meldet wenn der Leader länger als grenzwert an einer Stelle steht. (Zur
     * Überprüfung eines Testkriteriums)
     *
     * @param grenzwert
     */
//    private void checkStandingTime(int grenzwert) {
//        if (getDistanceRemaining() == 0 && stoppingTime == 0) {
//            stoppingTime = getTime();
//        } else if (getDistanceRemaining() != 0) {
//            stoppingTime = 0;
//        }
//        if (getTime() - stoppingTime > grenzwert) {
//            System.out.println("ich stehe zulange!");
//        }
//    }
}

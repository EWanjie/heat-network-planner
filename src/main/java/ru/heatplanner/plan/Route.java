package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;

import java.util.ArrayList;
import java.util.List;

/** Найденная трасса: цепочка прямых участков от старта (точки подключения ОКС) к месту врезки. */
public final class Route {

    /** Прямой участок трассы. */
    public static final class Leg {
        public final Coordinate from;
        public final Coordinate to;
        /** Специальный проход: один прямой участок с коэффициентом стоимости. */
        public final boolean special;
        public final double kSpec;
        /** Пересекаемые ограничения (для специального прохода). */
        public final List<Obstacle> crossed;
        /** Финальный прямой участок к самой точке подключения внутри здания ОКС. */
        public final boolean terminal;
        /** Заключительный подход к месту врезки (исключение отступа от существующей сети). */
        public final boolean tieInApproach;

        Leg(Coordinate from, Coordinate to, boolean special, double kSpec, List<Obstacle> crossed, boolean terminal,
            boolean tieInApproach) {
            this.from = from;
            this.to = to;
            this.special = special;
            this.kSpec = kSpec;
            this.crossed = crossed;
            this.terminal = terminal;
            this.tieInApproach = tieInApproach;
        }

        public double length() {
            return from.distance(to);
        }
    }

    public final List<Leg> legs;
    public final Start start;
    public final Goal goal;
    /** Длина по горизонтальной проекции, м. */
    public final double length;
    /** Стоимость труб по таблице 1 для ДУ поиска с коэффициентами специальных проходов, руб. */
    public final double cost;
    public final int dn;

    Route(List<Leg> legs, Start start, Goal goal, int dn) {
        this.legs = legs;
        this.start = start;
        this.goal = goal;
        this.dn = dn;
        double len = 0;
        double c = 0;
        double price = Rules.PRICE[Rules.indexOfDn(dn)];
        for (Leg l : legs) {
            len += l.length();
            c += l.length() * price * l.kSpec;
        }
        this.length = len;
        this.cost = c;
    }

    /** Вершины трассы в порядке от старта к врезке. */
    public List<Coordinate> vertices() {
        List<Coordinate> out = new ArrayList<>();
        for (int i = 0; i < legs.size(); i++) {
            if (i == 0) {
                out.add(legs.get(0).from);
            }
            out.add(legs.get(i).to);
        }
        return out;
    }

    /** Наибольший угол поворота между соседними участками, градусы. */
    public double maxTurnDeg() {
        double max = 0;
        for (int i = 1; i < legs.size(); i++) {
            max = Math.max(max, Angles.turnDeg(Angles.unit(legs.get(i - 1).from, legs.get(i - 1).to),
                    Angles.unit(legs.get(i).from, legs.get(i).to)));
        }
        return max;
    }
}

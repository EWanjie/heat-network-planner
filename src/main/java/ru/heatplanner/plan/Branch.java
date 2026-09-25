package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;

import java.util.ArrayList;
import java.util.List;

/**
 * Ветвь новой сети: трасса от точки подключения до места врезки. Врезка идёт либо в существующую сеть
 * (parent = -1), либо в другую ветвь этой же сети (parent — её номер): тогда в точке врезки строится новая камера.
 * Номер родителя всегда меньше номера ветви, поэтому порядок номеров — обход от корней к листьям.
 */
public final class Branch {

    /** Полезная нагрузка цели поиска, лежащей на другой ветви. */
    public static final class Point {
        public final int branch;

        Point(int branch) {
            this.branch = branch;
        }
    }

    private static final GeometryFactory F = new GeometryFactory();

    public final PlanInput.Target target;
    public final Route route;
    public final int parent;
    /** Врезка в существующую сеть (когда parent = -1). */
    public final Goals.Attach existing;
    public final Coordinate attach;
    /** Расстояние от начала родительской ветви до места врезки вдоль неё, м. */
    public final double parentPos;
    public final LineString line;
    public final double total;
    /** Расстояние от точки подключения до начала каждого участка; последний элемент — полная длина. */
    public final double[] legStart;

    Branch(PlanInput.Target target, Route route, int parent, Goals.Attach existing, Coordinate attach, double parentPos) {
        this.target = target;
        this.route = route;
        this.parent = parent;
        this.existing = existing;
        this.attach = attach;
        this.parentPos = parentPos;
        List<Coordinate> cs = new ArrayList<>(route.vertices());
        this.line = F.createLineString(cs.toArray(new Coordinate[0]));
        this.legStart = new double[route.legs.size() + 1];
        for (int i = 0; i < route.legs.size(); i++) {
            legStart[i + 1] = legStart[i] + route.legs.get(i).length();
        }
        this.total = legStart[route.legs.size()];
    }

    /** Номер участка, содержащего позицию pos (для границы — участок, начинающийся в ней, если он есть). */
    public int legAt(double pos) {
        for (int i = 0; i < route.legs.size(); i++) {
            if (pos < legStart[i + 1] - 1e-9) {
                return i;
            }
        }
        return route.legs.size() - 1;
    }

    public Coordinate pointAt(double pos) {
        return new LengthIndexedLine(line).extractPoint(Math.max(0, Math.min(total, pos)));
    }

    public double project(Coordinate p) {
        return new LengthIndexedLine(line).project(p);
    }

    /** Точку врезки другой ветви можно поставить здесь: обычный участок, не у самых концов. */
    public boolean canJoinAt(double pos) {
        if (pos < 0.5 || pos > total - 0.5) {
            return false;
        }
        Route.Leg leg = route.legs.get(legAt(pos));
        int i = legAt(pos);
        return !leg.special && !leg.terminal && pos - legStart[i] >= 0.5 && legStart[i + 1] - pos >= 0.5;
    }
}
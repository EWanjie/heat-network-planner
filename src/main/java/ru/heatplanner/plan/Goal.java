package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;

import java.util.Set;

/**
 * Место врезки в существующую сеть: точка на существующем участке (там появится новая камера) или существующая камера.
 * exempt — существующие участки, которых касается врезка: отступ до них на заключительном подходе не проверяется.
 */
public final class Goal {

    public final Coordinate xy;
    public final Set<Obstacle> exempt;
    /** Единичное направление существующей линии в точке врезки или null (камера на стыке участков). */
    public final double[] lineDirection;
    /** Что это: участок, камера — для последующего расчёта; поиску не нужно. */
    public final Object payload;

    public Goal(Coordinate xy, Set<Obstacle> exempt, double[] lineDirection, Object payload) {
        this.xy = xy;
        this.exempt = exempt;
        this.lineDirection = lineDirection;
        this.payload = payload;
    }
}

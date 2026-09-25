package ru.heatplanner.plan;

import org.locationtech.jts.geom.Geometry;

/** Пространственное ограничение в метрах (EPSG:32637) с правилом из таблицы 2. */
public final class Obstacle {

    public final int index;
    public final Id id;
    public final Rules.RestrictionRule rule;
    public final Geometry geometry;
    /** Собственный расчётный габарит (ширина), м: газопровод 0,40, кабель 0,20, теплосеть — по её ДУ; 0 — нет. */
    public final double envelopeWidth;
    /** Откуда объект: "input" — из файла, "OpenStreetMap" — дополнительные дороги, "existing_network" — участок сети. */
    public final String source;
    /** Дорога задана осью без ширины: границы полосы неизвестны, специальный проход через неё не рассчитать. */
    public final boolean widthUnknown;
    /** Ось дороги превращена в расчётный полигон по ширине (это модель, а не измеренная граница проезжей части). */
    public final boolean derivedFromCenterline;

    Obstacle(int index, Id id, Rules.RestrictionRule rule, Geometry geometry, double envelopeWidth, String source,
             boolean widthUnknown, boolean derivedFromCenterline) {
        this.index = index;
        this.id = id;
        this.rule = rule;
        this.geometry = geometry;
        this.envelopeWidth = envelopeWidth > 0 ? envelopeWidth : rule.envelopeWidth;
        this.source = source;
        this.widthUnknown = widthUnknown;
        this.derivedFromCenterline = derivedFromCenterline;
    }

    /** Ограничение можно пройти специальным проходом (а не только обойти). */
    public boolean crossable() {
        if (rule.kind == Rules.Kind.FORBIDDEN) {
            return false;
        }
        return !widthUnknown;
    }

    public boolean isExistingNetwork() {
        return "heat_network".equals(rule.type) && "existing_network".equals(source);
    }

    /**
     * Наибольшее допустимое расстояние от оси новой сети до геометрии ограничения, при котором ещё нарушается отступ:
     * отступ таблицы 2 плюс половина габарита ограничения плюс половина ширины пары труб выбранного ДУ.
     * Так расстояние измеряется между внешними границами двух габаритов (разъяснение 7).
     */
    public double axisDistance(int dn) {
        return rule.clearance(dn) + envelopeWidth / 2 + Rules.pairWidth(dn) / 2;
    }

    @Override
    public String toString() {
        return rule.type + " " + id;
    }
}

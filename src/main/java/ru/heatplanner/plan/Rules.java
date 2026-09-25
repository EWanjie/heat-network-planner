package ru.heatplanner.plan;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Правила технического приложения как данные: таблица 1 (диаметры), стоимость камер и врезок, штраф,
 * веса ранжирования и таблица 2 (пространственные ограничения). Алгоритм читает правила отсюда и не хранит
 * чисел у себя: так расчёт можно проверить по документу и не править при втором наборе данных.
 */
public final class Rules {

    private Rules() {
    }

    // ---- Таблица 1: диаметры ----------------------------------------------------------------------------------------
    public static final int[] DN = {50, 65, 80, 100, 125, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400};
    /** Пропускная способность, т/ч. */
    public static final double[] CAPACITY = {3.5, 8.3, 13.2, 22.3, 40.2, 65.1, 152.3, 274.9, 437.4, 943.1, 1663.4,
            2627.7, 3735.1, 5296.8, 7165.0, 9391.8, 15012.8, 22501.9};
    /** Предельная длина непрерывной части одного диаметра, м. */
    public static final double[] MAX_LENGTH = {181, 245, 327, 419, 554, 696, 1042, 1379, 1718, 2477, 3245,
            4037, 4775, 5644, 6518, 7419, 9288, 11276};
    /** Новое строительство, руб./м. */
    public static final double[] PRICE = {74023, 78631, 83530, 89748, 97275, 105507, 120275, 135323, 150022, 190299,
            224137, 264790, 324298, 325996, 327693, 418777, 428074, 683417};
    /** Расчётная ширина пары труб, м. */
    public static final double[] PAIR_WIDTH = {0.400, 0.430, 0.470, 0.510, 0.600, 0.650, 0.880, 1.050, 1.150, 1.370,
            1.670, 1.850, 2.050, 2.250, 2.450, 2.650, 3.100, 3.450};

    /** Индекс диаметра в таблице по значению ДУ, мм; -1, если такого ДУ нет. */
    public static int indexOfDn(int dn) {
        for (int i = 0; i < DN.length; i++) {
            if (DN[i] == dn) {
                return i;
            }
        }
        return -1;
    }

    /** Индекс наименьшего ДУ, пропускающего расход, или -1, если расход больше самого большого ДУ. */
    public static int indexForFlow(double flowTph) {
        for (int i = 0; i < DN.length; i++) {
            if (CAPACITY[i] + 1e-9 >= flowTph) {
                return i;
            }
        }
        return -1;
    }

    public static double pairWidth(int dn) {
        int i = indexOfDn(dn);
        if (i < 0) {
            throw new IllegalArgumentException("ДУ " + dn + " мм нет в таблице 1");
        }
        return PAIR_WIDTH[i];
    }

    // ---- Раздел 3.2 и 6: камеры, врезки, штраф, ранжирование ---------------------------------------------------------

    /** Стоимость новой камеры по наибольшему ДУ примыкающих участков (таблица раздела 3.2). */
    public static double chamberCost(int maxDn) {
        if (maxDn <= 200) {
            return 3_000_000;
        }
        if (maxDn <= 500) {
            return 5_000_000;
        }
        return maxDn <= 1000 ? 8_000_000 : 12_000_000;
    }

    /** Врезка в существующую камеру: за каждый новый линейный участок, заканчивающийся в ней. */
    public static final double EXISTING_CHAMBER_TIE_IN = 5_000_000;
    /** Расстояние, в пределах которого врезка идёт в существующую камеру, м. */
    public static final double CHAMBER_SNAP_M = 10.0;
    /** Не более четырёх линейных участков у камеры. */
    public static final int MAX_CHAMBER_SEGMENTS = 4;
    /** Наибольший угол поворота, градусы. */
    public static final double MAX_TURN_DEG = 90.0;

    public static double penalty(double flowTph) {
        return 100_000_000 + 500_000 * flowTph;
    }

    public static double score(double cost, double length) {
        return 0.7 * (cost / 25_000_000) + 0.3 * (length / 100);
    }

    // ---- Таблица 2: пространственные ограничения ----------------------------------------------------------------------

    /** Как новая сеть обходится с ограничением. */
    public enum Kind {
        /** Пересечение запрещено, нужен обход с отступом. */
        FORBIDDEN,
        /** Специальный проход через полигон: интервал от входа до выхода плюс extent с каждой стороны. */
        SPECIAL_AREA,
        /** Специальный проход через линию: интервал extent в обе стороны от точки пересечения. */
        SPECIAL_LINE
    }

    /**
     * Правило для одного типа ограничения.
     * clearance — минимальное расстояние от геометрии ограничения (или его собственного габарита) до внешней
     * границы расчётного габарита новой сети; для oks зависит от ДУ, поэтому считается методом clearance(dn).
     */
    public static final class RestrictionRule {
        public final String type;
        public final String title;
        public final Kind kind;
        private final double clearance;
        /** Минимальный угол пересечения, градусы; 0 — не задан. */
        public final double minAngleDeg;
        /** Длина специального интервала за границей полигона или в каждую сторону от точки пересечения, м. */
        public final double extent;
        public final double kSpec;
        /** Собственный расчётный габарит (ширина) ограничения, м; 0 — габарита нет. */
        public final double envelopeWidth;

        RestrictionRule(String type, String title, Kind kind, double clearance, double minAngleDeg, double extent,
                        double kSpec, double envelopeWidth) {
            this.type = type;
            this.title = title;
            this.kind = kind;
            this.clearance = clearance;
            this.minAngleDeg = minAngleDeg;
            this.extent = extent;
            this.kSpec = kSpec;
            this.envelopeWidth = envelopeWidth;
        }

        /** Минимальный просвет при прохождении рядом, м. Для oks — по ДУ новой сети (5 / 7 / 9 м). */
        public double clearance(int dn) {
            if ("oks".equals(type)) {
                return dn < 500 ? 5.0 : (dn <= 800 ? 7.0 : 9.0);
            }
            return clearance;
        }
    }

    /** Таблица 2 в порядке приложения. Ключ — restriction_type. */
    public static final Map<String, RestrictionRule> RESTRICTIONS = new LinkedHashMap<>();

    static {
        put(new RestrictionRule("oks", "Существующий ОКС", Kind.FORBIDDEN, 5.0, 0, 0, 1, 0));
        put(new RestrictionRule("park", "Парк", Kind.FORBIDDEN, 1.0, 0, 0, 1, 0));
        put(new RestrictionRule("social_area", "Территория социального объекта", Kind.FORBIDDEN, 1.0, 0, 0, 1, 0));
        put(new RestrictionRule("prohibited_site", "Запрещённая территория", Kind.FORBIDDEN, 1.0, 0, 0, 1, 0));
        put(new RestrictionRule("water", "Водный объект", Kind.FORBIDDEN, 1.0, 0, 0, 1, 0));
        put(new RestrictionRule("railway", "Железная дорога", Kind.FORBIDDEN, 1.0, 0, 0, 1, 0));
        put(new RestrictionRule("road", "Автомобильная дорога", Kind.SPECIAL_AREA, 1.5, 45, 3.0, 1.60, 0));
        put(new RestrictionRule("tram_tracks", "Трамвайные пути", Kind.SPECIAL_AREA, 1.5, 45, 3.0, 1.75, 0));
        put(new RestrictionRule("gas_pipeline", "Газопровод", Kind.SPECIAL_LINE, 2.0, 0, 2.0, 1.25, 0.40));
        put(new RestrictionRule("power_cable", "Силовой кабель до 35 кВ", Kind.SPECIAL_LINE, 2.0, 0, 2.0, 1.15, 0.20));
        // Габарит существующей теплосети задаётся её диаметром (таблица 1), поэтому envelopeWidth здесь 0 и
        // берётся у самого объекта.
        put(new RestrictionRule("heat_network", "Существующая тепловая сеть (пересечение без врезки)", Kind.SPECIAL_LINE,
                1.0, 0, 2.0, 1.05, 0));
    }

    private static void put(RestrictionRule rule) {
        RESTRICTIONS.put(rule.type, rule);
    }
}

package ru.heatplanner;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Все правила технического приложения в одном месте, как данные: таблицы диаметров и стоимости (4.1),
 * шкала камер и врезки (8.2), штраф (8.3), веса ранжирования (9) и правила по типам ограничений (5.1).
 *
 * Правила не «зашиты» в алгоритм: поиск трасс, стоимость и проверки читают их отсюда,
 * а сервис отдаёт их же на экран («Как это посчитано»), поэтому расчёт объясним и проверяем.
 */
final class Rules {

    private Rules() {
    }

    // ---- Таблица 4.1: диаметры, пропускная способность, предельная длина, стоимость 1 м -------------------------
    static final int[] DN = {50, 65, 80, 100, 125, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400};
    static final double[] CAPACITY = {3.5, 8.3, 13.2, 22.3, 40.2, 65.1, 152.3, 274.9, 437.4, 943.1, 1663.4,
            2627.7, 3735.1, 5296.8, 7165.0, 9391.8, 15012.8, 22501.9};
    static final int[] MAX_LENGTH = {181, 245, 327, 419, 554, 696, 1042, 1379, 1718, 2477, 3245,
            4037, 4775, 5644, 6518, 7419, 9288, 11276};
    static final double[] COST_NEW = {74023, 78631, 83530, 89748, 97275, 105507, 120275, 135323, 150022, 190299,
            224137, 264790, 324298, 325996, 327693, 418777, 428074, 683417};
    static final double[] COST_RECON = {96180, 109989, 117582, 133694, 148030, 152295, 181766, 202030, 228707,
            271317, 333884, 372703, 439571, 489918, 553607, 606679, 825692, 978584};

    // ---- Раздел 8: врезки, камеры, штраф --------------------------------------------------------------------------
    static final double TIE_IN_COST = 5_000_000;
    /** Расстояние, в пределах которого врезка выполняется в существующей камере (8.2). */
    static final double CHAMBER_SNAP_M = 10.0;
    /** Не более четырёх участков у камеры: один к источнику и три остальных (раздел 3). */
    static final int MAX_CHAMBER_SEGMENTS = 4;
    static final double PENALTY_FIXED = 100_000_000;
    static final double PENALTY_PER_TPH = 500_000;

    // ---- Раздел 9: ранжирование -----------------------------------------------------------------------------------
    static final double COST_SCALE = 25_000_000;
    static final double LENGTH_SCALE = 100;
    static final double WEIGHT_COST = 0.7;
    static final double WEIGHT_LENGTH = 0.3;

    /** Индекс наименьшего диаметра, пропускающего расход, или -1, если расход больше самого большого диаметра. */
    static int indexForFlow(double flowTph) {
        for (int i = 0; i < DN.length; i++) {
            if (CAPACITY[i] + 1e-9 >= flowTph) {
                return i;
            }
        }
        return -1;
    }

    static int indexOfDn(int dn) {
        for (int i = 0; i < DN.length; i++) {
            if (DN[i] >= dn) {
                return i;
            }
        }
        return DN.length - 1;
    }

    /** Шкала стоимости камеры по наибольшему диаметру (8.2); общая для новой и реконструируемой камеры. */
    static double chamberCost(int maxDn) {
        if (maxDn <= 200) {
            return 3_000_000;
        }
        if (maxDn <= 500) {
            return 5_000_000;
        }
        if (maxDn <= 1000) {
            return 8_000_000;
        }
        return 12_000_000;
    }

    static double penalty(double flowTph) {
        return PENALTY_FIXED + PENALTY_PER_TPH * flowTph;
    }

    /** Итоговый показатель S (раздел 9); чем меньше, тем лучше. */
    static double score(double cost, double length) {
        return WEIGHT_COST * cost / COST_SCALE + WEIGHT_LENGTH * length / LENGTH_SCALE;
    }

    /**
     * Правило для одного типа пространственного ограничения (таблица 5.1).
     * forbidden — пересекать нельзя, надо обойти с зазором minDistance;
     * иначе допускается специальный проход: участок внутри зоны дороже в kSpec раз.
     */
    static final class Rule {
        final String type;
        final String title;
        final boolean forbidden;
        /** Минимальное горизонтальное расстояние, м (для существующего ОКС зависит от диаметра — см. oksDistance). */
        final double minDistance;
        final double kSpec;
        /** Длина специального участка за границей зоны (полигон) или в каждую сторону от точки пересечения (линия), м. */
        final double extent;
        /** true — зона это полигон (дорога, трамвай); false — линейный объект (газ, кабель, теплосеть). */
        final boolean areaZone;
        /** Минимальный угол пересечения, градусы; 0 — не задаётся. */
        final double minAngleDeg;
        /** false — тип не описан в таблице 5.1, принято правило «пересечение запрещено, зазор 1 м». */
        final boolean known;

        Rule(String type, String title, boolean forbidden, double minDistance, double kSpec, double extent,
             boolean areaZone, double minAngleDeg, boolean known) {
            this.type = type;
            this.title = title;
            this.forbidden = forbidden;
            this.minDistance = minDistance;
            this.kSpec = kSpec;
            this.extent = extent;
            this.areaZone = areaZone;
            this.minAngleDeg = minAngleDeg;
            this.known = known;
        }
    }

    /** Правила по типам, в порядке таблицы 5.1. Ключ — restriction_type (или "oks" для существующих ОКС). */
    static final Map<String, Rule> RULES = new LinkedHashMap<>();

    static {
        put(new Rule("oks", "Существующий ОКС", true, 5.0, 1, 0, true, 0, true));
        put(new Rule("park", "Парк", true, 1.0, 1, 0, true, 0, true));
        put(new Rule("social_area", "Территория социального объекта", true, 1.0, 1, 0, true, 0, true));
        put(new Rule("prohibited_site", "Запрещённая территория", true, 1.0, 1, 0, true, 0, true));
        put(new Rule("water", "Водный объект", true, 1.0, 1, 0, true, 0, true));
        put(new Rule("road", "Автомобильная дорога", false, 1.5, 1.60, 3.0, true, 45, true));
        put(new Rule("tram_tracks", "Трамвайные пути", false, 1.5, 1.75, 3.0, true, 45, true));
        put(new Rule("gas_pipeline", "Газопровод", false, 2.0, 1.25, 2.0, false, 0, true));
        put(new Rule("power_cable", "Силовой кабель до 35 кВ", false, 2.0, 1.15, 2.0, false, 0, true));
        put(new Rule("heat_network", "Существующая тепловая сеть (пересечение без врезки)", false, 1.0, 1.05, 2.0,
                false, 0, true));
    }

    private static void put(Rule rule) {
        RULES.put(rule.type, rule);
    }

    /** Правило для типа ограничения; для незнакомого типа — осторожное «запрещено» (и known = false). */
    static Rule forType(String restrictionType) {
        String key = "oks_existing".equals(restrictionType) ? "oks" : restrictionType;
        Rule rule = RULES.get(key);
        if (rule != null) {
            return rule;
        }
        return new Rule(restrictionType, "Тип «" + restrictionType + "» (нет в таблице 5.1)",
                true, 1.0, 1, 0, true, 0, false);
    }

    /** Зазор до существующего ОКС зависит от диаметра новой сети (таблица 5.1). */
    static double oksDistance(int dn) {
        if (dn < 500) {
            return 5.0;
        }
        return dn <= 800 ? 7.0 : 9.0;
    }
}

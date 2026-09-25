package ru.heatplanner.plan;

/**
 * Настройки расчёта, которые не являются нормами приложения: вычислительный запас, точность аппроксимации
 * буферов и трактовки, для которых приложение и разъяснения не дают однозначного ответа.
 * Все трактовки задаются здесь явно, а не спрятаны в алгоритме поиска.
 */
public final class RuleSet {

    /**
     * Как проверять отступ от линейного объекта (газопровод, кабель, существующая теплосеть) сразу за границей
     * его специального интервала.
     *
     * Специальный интервал по приложению — по 2 м в каждую сторону от точки пересечения, а отступ вне него —
     * 2 м (газ, кабель) или 1 м (теплосеть) от габарита плюс половина ширины трубы. Для трассы, идущей поперёк,
     * ось на границе интервала находится в 2 м от линии, что меньше требуемого. Приложение и разъяснения не
     * говорят, как это разрешается.
     */
    public enum LineCrossingMode {
        /** Строго по приложению: такое пересечение — конфликт правил и допустимым не считается. */
        STRICT,
        /** Допущение: отступ от пересекаемого линейного объекта не проверяется на самом прямом переходе. */
        EXEMPT_ON_CROSSING
    }

    /**
     * Как выбирать подход к точке подключения внутри здания ОКС.
     * По приложению допускается один финальный прямой участок от ближайшей к точке границы полигона до самой точки;
     * заменять его входом с дальней стороны без основания в правилах нельзя.
     */
    public enum OwnApproachMode {
        /** Строго: только ближайшая граница (при равном расстоянии — все такие границы). */
        NEAREST_ONLY,
        /**
         * Если подход к ближайшей границе невозможен (она во внутреннем дворе или перекрыта другими ограничениями),
         * берётся ближайшая из остальных допустимых границ. Использование такого подхода отмечается в результате.
         */
        NEAREST_VALID
    }

    /** Что делать с дорогой, заданной осевой линией без ширины (в том числе из OpenStreetMap). */
    public enum RoadWidthPolicy {
        /** Ширину не придумывать: пересечение такой дороги не считается допустимым. */
        REQUIRE_WIDTH
    }

    /** Вычислительный запас к отступам, м. Не норматив: закрывает погрешность округления и аппроксимации дуг буфера. */
    public final double geometryEps;
    /** Число сегментов на четверть окружности в буфере (чем больше, тем меньше срезаются дуги). */
    public final int bufferQuadrantSegments;
    public final LineCrossingMode lineCrossing;
    public final RoadWidthPolicy roadWidth;
    /**
     * Наименьший острый угол между заключительным подходом и существующей линией в точке врезки, градусы.
     * Приложение снимает отступ до существующей сети на подходе к врезке, но не разрешает идти вдоль трубы:
     * пологий подход шёл бы внутри отступа на большой длине. Числа в приложении нет, это параметр проекта.
     */
    public final double tieInApproachMinAngleDeg;
    public final OwnApproachMode ownApproach;

    public RuleSet(double geometryEps, int bufferQuadrantSegments, LineCrossingMode lineCrossing,
                   RoadWidthPolicy roadWidth, double tieInApproachMinAngleDeg, OwnApproachMode ownApproach) {
        this.geometryEps = geometryEps;
        this.bufferQuadrantSegments = bufferQuadrantSegments;
        this.lineCrossing = lineCrossing;
        this.roadWidth = roadWidth;
        this.tieInApproachMinAngleDeg = tieInApproachMinAngleDeg;
        this.ownApproach = ownApproach;
    }

    /** Строгие настройки: отступы и пересечения ровно по приложению, ширина дороги обязательна. */
    public static RuleSet strict() {
        return new RuleSet(0.01, 32, LineCrossingMode.STRICT, RoadWidthPolicy.REQUIRE_WIDTH, 45, OwnApproachMode.NEAREST_ONLY);
    }

    /** Те же правила, но с запасным подходом к ближайшей допустимой границе здания. */
    public RuleSet withOwnApproach(OwnApproachMode mode) {
        return new RuleSet(geometryEps, bufferQuadrantSegments, lineCrossing, roadWidth, tieInApproachMinAngleDeg, mode);
    }
}

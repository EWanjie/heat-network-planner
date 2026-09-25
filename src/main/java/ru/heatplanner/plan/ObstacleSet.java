package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.buffer.BufferParameters;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Набор ограничений с готовыми запретными зонами.
 *
 * Запретная зона ограничения для диаметра dn — его геометрия, расширенная на axisDistance(dn): любая ось новой сети,
 * попавшая внутрь зоны, нарушает отступ. Зоны разные для каждого ДУ (ширина пары и отступ до ОКС зависят от него),
 * поэтому считаются по требованию и кэшируются по паре (ограничение, ДУ). Пространственный индекс отсеивает
 * далёкие ограничения, а после него выполняется точная проверка по всей геометрии отрезка, а не по концам.
 */
public final class ObstacleSet {

    /** Запретная зона: сама геометрия и её подготовленная копия для быстрых проверок. */
    public static final class Zone {
        public final Geometry geometry;
        final PreparedGeometry prepared;

        Zone(Geometry geometry) {
            this.geometry = geometry;
            this.prepared = PreparedGeometryFactory.prepare(geometry);
        }
    }

    private final List<Obstacle> obstacles;
    private final RuleSet rules;
    private final GeometryFactory factory = new GeometryFactory();
    private final STRtree index = new STRtree();
    private final ConcurrentHashMap<Long, Zone> zones = new ConcurrentHashMap<>();
    /** Наибольший axisDistance среди ограничений для каждого ДУ: радиус запроса к индексу. */
    private final double[] reach = new double[Rules.DN.length];

    public ObstacleSet(List<Obstacle> obstacles, RuleSet rules) {
        this.obstacles = new ArrayList<>(obstacles);
        this.rules = rules;
        for (Obstacle o : this.obstacles) {
            index.insert(o.geometry.getEnvelopeInternal(), o);
            for (int i = 0; i < Rules.DN.length; i++) {
                reach[i] = Math.max(reach[i], o.axisDistance(Rules.DN[i]));
            }
        }
        index.build();
    }

    public List<Obstacle> all() {
        return obstacles;
    }

    /** Ограничения, чья запретная зона для ДУ dn может задеть область env. */
    @SuppressWarnings("unchecked")
    public List<Obstacle> near(Envelope env, int dn) {
        Envelope query = new Envelope(env);
        query.expandBy(reach[Rules.indexOfDn(dn)] + rules.geometryEps);
        return (List<Obstacle>) index.query(query);
    }

    /**
     * Запретная зона ограничения для ДУ dn: расширение геометрии на axisDistance плюс вычислительный запас.
     * Дуги буфера аппроксимируются хордами внутри настоящей окружности; при 32 сегментах на четверть срез не
     * превышает 0,03 % радиуса, то есть остаётся меньше запаса.
     */
    public Zone zone(Obstacle o, int dn) {
        long key = ((long) o.index << 8) | Rules.indexOfDn(dn);
        return zones.computeIfAbsent(key, k -> {
            BufferParameters p = new BufferParameters(rules.bufferQuadrantSegments, BufferParameters.CAP_ROUND,
                    BufferParameters.JOIN_ROUND, 5.0);
            return new Zone(BufferOp.bufferOp(o.geometry, o.axisDistance(dn) + rules.geometryEps, p));
        });
    }

    // ---- Быстрый предфильтр «заведомо занято» ---------------------------------------------------------------------

    /** Размер ячейки растра, м. Ячейка считается занятой, только если она целиком внутри запретной зоны. */
    private static final double CELL_M = 1.0;
    /** Шаг проверки вдоль отрезка, м: вдвое меньше ячейки, поэтому занятую ячейку отрезок не пропускает. */
    private static final double SAMPLE_M = 0.5;
    /** Половина диагонали ячейки с запасом: центр глубже этого внутри зоны — вся ячейка внутри. */
    private static final double SHRINK_M = 0.75;

    /** Растр занятых ячеек для одного ДУ: бит стоит, если ячейка целиком внутри какой-либо запретной зоны. */
    private static final class Raster {
        final double x0;
        final double y0;
        final int nx;
        final int ny;
        final BitSet blocked;

        Raster(double x0, double y0, int nx, int ny) {
            this.x0 = x0;
            this.y0 = y0;
            this.nx = nx;
            this.ny = ny;
            this.blocked = new BitSet(nx * ny);
        }
    }

    private final ConcurrentHashMap<Integer, Raster> rasters = new ConcurrentHashMap<>();

    private Raster raster(int dn) {
        return rasters.computeIfAbsent(dn, k -> {
            Envelope all = new Envelope();
            List<Zone> list = new ArrayList<>();
            for (Obstacle o : obstacles) {
                Zone z = zone(o, dn);
                list.add(z);
                all.expandToInclude(z.geometry.getEnvelopeInternal());
            }
            if (all.isNull()) {
                return new Raster(0, 0, 1, 1);
            }
            int nx = (int) Math.ceil(all.getWidth() / CELL_M) + 1;
            int ny = (int) Math.ceil(all.getHeight() / CELL_M) + 1;
            Raster r = new Raster(all.getMinX(), all.getMinY(), nx, ny);
            for (Zone z : list) {
                Geometry core = z.geometry.buffer(-SHRINK_M);
                if (core.isEmpty()) {
                    continue;
                }
                PreparedGeometry prepared = PreparedGeometryFactory.prepare(core);
                Envelope e = core.getEnvelopeInternal();
                int ix0 = Math.max(0, (int) Math.floor((e.getMinX() - r.x0) / CELL_M));
                int ix1 = Math.min(nx - 1, (int) Math.floor((e.getMaxX() - r.x0) / CELL_M));
                int iy0 = Math.max(0, (int) Math.floor((e.getMinY() - r.y0) / CELL_M));
                int iy1 = Math.min(ny - 1, (int) Math.floor((e.getMaxY() - r.y0) / CELL_M));
                for (int iy = iy0; iy <= iy1; iy++) {
                    for (int ix = ix0; ix <= ix1; ix++) {
                        if (prepared.intersects(factory.createPoint(
                                new Coordinate(r.x0 + (ix + 0.5) * CELL_M, r.y0 + (iy + 0.5) * CELL_M)))) {
                            r.blocked.set(iy * nx + ix);
                        }
                    }
                }
            }
            return r;
        });
    }

    /** Отрезок ab заведомо проходит по занятой ячейке (быстрая проверка; «нет» не значит «свободно»). */
    public boolean certainlyBlocked(Coordinate a, Coordinate b, int dn) {
        Raster r = raster(dn);
        double len = a.distance(b);
        int n = Math.max(1, (int) Math.ceil(len / SAMPLE_M));
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            int ix = (int) Math.floor((a.x + t * (b.x - a.x) - r.x0) / CELL_M);
            int iy = (int) Math.floor((a.y + t * (b.y - a.y) - r.y0) / CELL_M);
            if (ix >= 0 && iy >= 0 && ix < r.nx && iy < r.ny && r.blocked.get(iy * r.nx + ix)) {
                return true;
            }
        }
        return false;
    }

    /** Ось отрезка ab (расчётная линия новой сети) не входит ни в одну запретную зону, кроме исключённых. */
    public boolean segmentClear(Coordinate a, Coordinate b, int dn, Set<Obstacle> exempt) {
        // Без исключений занятые ячейки растра дают ответ «нельзя» без точной проверки.
        if ((exempt == null || exempt.isEmpty()) && certainlyBlocked(a, b, dn)) {
            return false;
        }
        return firstViolation(a, b, dn, exempt) == null;
    }

    /** Первое ограничение, отступ до которого нарушает отрезок ab, или null, если нарушений нет. */
    public Obstacle firstViolation(Coordinate a, Coordinate b, int dn, Set<Obstacle> exempt) {
        LineString segment = factory.createLineString(new Coordinate[]{a, b});
        for (Obstacle o : near(segment.getEnvelopeInternal(), dn)) {
            if (exempt != null && exempt.contains(o)) {
                continue;
            }
            if (zone(o, dn).prepared.intersects(segment)) {
                return o;
            }
        }
        return null;
    }

    /** Точка p лежит в запретной зоне какого-либо ограничения для ДУ dn (кроме исключённых). */
    public boolean pointBlocked(Coordinate p, int dn, Set<Obstacle> exempt) {
        Geometry point = factory.createPoint(p);
        for (Obstacle o : near(point.getEnvelopeInternal(), dn)) {
            if ((exempt == null || !exempt.contains(o)) && zone(o, dn).prepared.intersects(point)) {
                return true;
            }
        }
        return false;
    }
}

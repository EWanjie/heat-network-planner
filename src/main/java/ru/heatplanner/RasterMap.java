package ru.heatplanner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Растровая «карта проходимости» участка: сетка ячеек в метрах, где запретные зоны заблокированы (с запасом на зазор),
 * а зоны специального прохода дороже. По ней Router ищет пути; точные проверки и стоимость потом считаются
 * по настоящей геометрии, а не по ячейкам.
 */
final class RasterMap {

    static final byte FREE = 0;
    /** Ячейка в зазоре вокруг запретной зоны. */
    static final byte BUFFER = 1;
    /** Ячейка внутри запретной зоны. */
    static final byte INSIDE = 2;

    /** Не больше стольких ячеек: грубее сетка на больших территориях, но расчёт остаётся быстрым. */
    private static final int TARGET_CELLS = 450_000;
    private static final double MARGIN_M = 100.0;

    final double x0;
    final double y0;
    final double cell;
    final int nx;
    final int ny;
    final byte[] block;
    final float[] mult;

    private RasterMap(double x0, double y0, double cell, int nx, int ny) {
        this.x0 = x0;
        this.y0 = y0;
        this.cell = cell;
        this.nx = nx;
        this.ny = ny;
        this.block = new byte[nx * ny];
        this.mult = new float[nx * ny];
        Arrays.fill(mult, 1f);
    }

    /** Строит карту по границам данных и растеризует все ограничения. */
    static RasterMap build(PlanModel m) {
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        List<double[]> pts = new ArrayList<>();
        m.sources.values().forEach(s -> pts.add(s.xy));
        m.chambers.values().forEach(c -> pts.add(c.xy));
        m.segments.values().forEach(s -> {
            for (double[] p : s.pts) {
                pts.add(p);
            }
        });
        m.targets.forEach(t -> t.points.forEach(p -> pts.add(p.xy)));
        for (double[] p : pts) {
            minX = Math.min(minX, p[0]);
            minY = Math.min(minY, p[1]);
            maxX = Math.max(maxX, p[0]);
            maxY = Math.max(maxY, p[1]);
        }
        minX -= MARGIN_M;
        minY -= MARGIN_M;
        maxX += MARGIN_M;
        maxY += MARGIN_M;
        double cell = Math.max(1.0, Math.sqrt((maxX - minX) * (maxY - minY) / TARGET_CELLS));
        int nx = (int) Math.ceil((maxX - minX) / cell);
        int ny = (int) Math.ceil((maxY - minY) / cell);
        RasterMap map = new RasterMap(minX, minY, cell, nx, ny);
        for (PlanModel.Shape shape : m.shapes) {
            map.rasterize(shape);
        }
        return map;
    }

    int cellX(double x) {
        return (int) Math.floor((x - x0) / cell);
    }

    int cellY(double y) {
        return (int) Math.floor((y - y0) / cell);
    }

    boolean inBounds(int ix, int iy) {
        return ix >= 0 && iy >= 0 && ix < nx && iy < ny;
    }

    /** Номер ячейки, содержащей точку, или -1, если точка вне карты. */
    int indexOf(double x, double y) {
        int ix = cellX(x);
        int iy = cellY(y);
        return inBounds(ix, iy) ? iy * nx + ix : -1;
    }

    double centerX(int idx) {
        return x0 + (idx % nx + 0.5) * cell;
    }

    double centerY(int idx) {
        return y0 + (idx / nx + 0.5) * cell;
    }

    private interface CellOp {
        void apply(int idx);
    }

    private void rasterize(PlanModel.Shape shape) {
        Rules.Rule rule = shape.rule;
        // Быстрый отказ: фигура целиком вне карты (с учётом зазора).
        double reach = rule.forbidden ? rule.minDistance + cell : Math.max(rule.extent, 1.0) + cell;
        if (shape.maxX + reach < x0 || shape.minX - reach > x0 + nx * cell
                || shape.maxY + reach < y0 || shape.minY - reach > y0 + ny * cell) {
            return;
        }
        if (rule.forbidden) {
            // Запас 0,7 ячейки: путь по центрам ячеек не должен «срезать» настоящий зазор.
            double r = rule.minDistance + 0.7 * cell;
            if (shape.area) {
                fillInterior(shape.rings, idx -> block[idx] = INSIDE);
            }
            edgesNear(shape, r, idx -> {
                if (block[idx] == FREE) {
                    block[idx] = BUFFER;
                }
            });
        } else {
            float k = (float) rule.kSpec;
            CellOp raise = idx -> {
                if (mult[idx] < k) {
                    mult[idx] = k;
                }
            };
            if (shape.area) {
                fillInterior(shape.rings, raise);
            }
            edgesNear(shape, Math.max(rule.extent, 0.5 * cell), raise);
        }
    }

    /** Все ячейки, чей центр ближе r к любой стороне фигуры. */
    private void edgesNear(PlanModel.Shape shape, double r, CellOp op) {
        for (double[][] ring : shape.rings) {
            for (int i = 1; i < ring.length; i++) {
                double[] a = ring[i - 1];
                double[] b = ring[i];
                int ixMin = Math.max(0, cellX(Math.min(a[0], b[0]) - r));
                int ixMax = Math.min(nx - 1, cellX(Math.max(a[0], b[0]) + r));
                int iyMin = Math.max(0, cellY(Math.min(a[1], b[1]) - r));
                int iyMax = Math.min(ny - 1, cellY(Math.max(a[1], b[1]) + r));
                for (int iy = iyMin; iy <= iyMax; iy++) {
                    double cy = y0 + (iy + 0.5) * cell;
                    for (int ix = ixMin; ix <= ixMax; ix++) {
                        double cx = x0 + (ix + 0.5) * cell;
                        if (Geo.pointSegment(cx, cy, a, b) <= r) {
                            op.apply(iy * nx + ix);
                        }
                    }
                }
            }
        }
    }

    /** Заливка внутренности полигона по строкам сетки (правило чётности, дырки работают сами). */
    private void fillInterior(double[][][] rings, CellOp op) {
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (double[][] ring : rings) {
            for (double[] p : ring) {
                minY = Math.min(minY, p[1]);
                maxY = Math.max(maxY, p[1]);
            }
        }
        int iyMin = Math.max(0, cellY(minY));
        int iyMax = Math.min(ny - 1, cellY(maxY));
        List<Double> xs = new ArrayList<>();
        for (int iy = iyMin; iy <= iyMax; iy++) {
            double y = y0 + (iy + 0.5) * cell;
            xs.clear();
            for (double[][] ring : rings) {
                for (int i = 0, j = ring.length - 1; i < ring.length; j = i++) {
                    if ((ring[i][1] > y) != (ring[j][1] > y)) {
                        xs.add((ring[j][0] - ring[i][0]) * (y - ring[i][1]) / (ring[j][1] - ring[i][1]) + ring[i][0]);
                    }
                }
            }
            xs.sort(null);
            for (int k = 0; k + 1 < xs.size(); k += 2) {
                int from = Math.max(0, (int) Math.ceil((xs.get(k) - x0) / cell - 0.5));
                int to = Math.min(nx - 1, (int) Math.floor((xs.get(k + 1) - x0) / cell - 0.5));
                for (int ix = from; ix <= to; ix++) {
                    op.apply(iy * nx + ix);
                }
            }
        }
    }
}

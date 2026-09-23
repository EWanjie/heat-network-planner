package ru.heatplanner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Поиск трасс по растровой карте: алгоритм Дейкстры от точки подключения ОКС по всей сетке сразу
 * (потом из любой ячейки можно восстановить путь до неё), затем «выпрямление» пути: лишние повороты
 * убираются, если прямая между двумя точками пути свободна и не дороже исходного участка.
 *
 * Один Router — один поток: у него свои копии изменяемых массивов (блокировки и занятость).
 */
final class Router {

    private static final float INF = Float.MAX_VALUE;
    private static final double SQRT2 = Math.sqrt(2);
    private static final int[] DX = {1, -1, 0, 0, 1, 1, -1, -1};
    private static final int[] DY = {0, 0, 1, -1, 1, -1, 1, -1};

    final RasterMap map;
    /** Собственная копия блокировок: вокруг стартовой точки зазор временно снимается. */
    private final byte[] block;
    /** Ячейки, занятые уже проложенными новыми трубами: их нельзя пересекать, к ним можно только присоединиться. */
    private final byte[] occ;
    /** Собственная копия множителей стоимости: внутри здания самого ОКС путь временно почти бесплатный (труба там всё равно не прокладывается). */
    private final float[] mult;
    private final List<Integer> multChanged = new ArrayList<>();
    private final float[] dist;
    private final int[] prev;
    private long[] heap = new long[1 << 16];
    private int heapSize;
    private final List<int[]> unblocked = new ArrayList<>();
    private int start = -1;

    Router(RasterMap map) {
        this.map = map;
        this.block = map.block.clone();
        this.mult = map.mult.clone();
        this.occ = new byte[map.block.length];
        this.dist = new float[map.block.length];
        this.prev = new int[map.block.length];
    }

    // ---- Занятость ------------------------------------------------------------------------------------------------

    void clearOccupancy() {
        Arrays.fill(occ, (byte) 0);
    }

    /** Помечает ячейки вдоль ломаной как занятые (все ячейки, которых касается линия). */
    void occupy(double[][] pts) {
        double step = map.cell * 0.3;
        for (int i = 1; i < pts.length; i++) {
            double len = Geo.dist(pts[i - 1], pts[i]);
            int n = Math.max(1, (int) Math.ceil(len / step));
            for (int k = 0; k <= n; k++) {
                double t = (double) k / n;
                int idx = map.indexOf(pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
                        pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]));
                if (idx >= 0) {
                    occ[idx] = 1;
                }
            }
        }
    }

    // ---- Поиск ----------------------------------------------------------------------------------------------------

    /**
     * Готовит поиск от точки: снимает блокировки в круге радиуса exemptR (точка подключения часто стоит у самого
     * здания, внутри его зазора). Возвращает false, если точка вне карты.
     */
    boolean begin(double[] xy, double exemptR, List<PlanModel.Shape> own, PlanModel model) {
        restore();
        int idx = map.indexOf(xy[0], xy[1]);
        if (idx < 0) {
            return false;
        }
        start = idx;
        for (PlanModel.Shape shape : own) {
            releaseOwn(shape, model, own);
        }
        int r = (int) Math.ceil(exemptR / map.cell);
        int cx = idx % map.nx;
        int cy = idx / map.nx;
        for (int iy = cy - r; iy <= cy + r; iy++) {
            for (int ix = cx - r; ix <= cx + r; ix++) {
                if (map.inBounds(ix, iy) && Math.hypot(ix - cx, iy - cy) * map.cell <= exemptR + map.cell) {
                    int j = iy * map.nx + ix;
                    if (block[j] != RasterMap.FREE) {
                        unblocked.add(new int[]{j, block[j]});
                        block[j] = RasterMap.FREE;
                    }
                }
            }
        }
        return true;
    }

    /**
     * Контур, внутри которого стоит точка подключения, — здание самого ОКС: его внутренность и зазор вокруг
     * для этого ОКС не запрещены. Ячейки, которые запрещены ещё и другим объектом, остаются заблокированными.
     */
    private void releaseOwn(PlanModel.Shape own, PlanModel model, List<PlanModel.Shape> allOwn) {
        double reach = own.rule.minDistance + 0.7 * map.cell;
        int ixMin = Math.max(0, map.cellX(own.minX - reach));
        int ixMax = Math.min(map.nx - 1, map.cellX(own.maxX + reach));
        int iyMin = Math.max(0, map.cellY(own.minY - reach));
        int iyMax = Math.min(map.ny - 1, map.cellY(own.maxY + reach));
        for (int iy = iyMin; iy <= iyMax; iy++) {
            for (int ix = ixMin; ix <= ixMax; ix++) {
                int j = iy * map.nx + ix;
                if (block[j] == RasterMap.FREE) {
                    continue;
                }
                double x = map.x0 + (ix + 0.5) * map.cell;
                double y = map.y0 + (iy + 0.5) * map.cell;
                boolean ours = Geo.inside(x, y, own.rings) || Geo.pointBoundary(x, y, own.rings) <= reach;
                if (!ours || blockedByOther(x, y, model, allOwn)) {
                    continue;
                }
                unblocked.add(new int[]{j, block[j]});
                block[j] = RasterMap.FREE;
                if (Geo.inside(x, y, own.rings)) {
                    // Внутри здания трубы не будет: путь дойдёт до стены, дальше он обрезается (см. Planner.trimAtOwn).
                    multChanged.add(j);
                    mult[j] = 0.01f;
                }
            }
        }
    }

    private boolean blockedByOther(double x, double y, PlanModel model, List<PlanModel.Shape> allOwn) {
        for (PlanModel.Shape s : model.shapes) {
            if (!s.rule.forbidden || allOwn.contains(s)) {
                continue;
            }
            double r = s.rule.minDistance + 0.7 * map.cell;
            if (x < s.minX - r || x > s.maxX + r || y < s.minY - r || y > s.maxY + r) {
                continue;
            }
            if ((s.area && Geo.inside(x, y, s.rings)) || Geo.pointBoundary(x, y, s.rings) <= r) {
                return true;
            }
        }
        return false;
    }

    /** Возвращает снятые блокировки на место. */
    void restore() {
        for (int[] u : unblocked) {
            block[u[0]] = (byte) u[1];
        }
        unblocked.clear();
        for (int j : multChanged) {
            mult[j] = map.mult[j];
        }
        multChanged.clear();
    }

    /** Дейкстра из стартовой ячейки. Занятые ячейки получают расстояние, но дальше по ним путь не идёт. */
    void run() {
        Arrays.fill(dist, INF);
        Arrays.fill(prev, -1);
        heapSize = 0;
        dist[start] = 0;
        push(0f, start);
        int nx = map.nx;
        int ny = map.ny;
        double cell = map.cell;
        while (heapSize > 0) {
            long top = pop();
            float d = Float.intBitsToFloat((int) (top >>> 32));
            int i = (int) (top & 0xffffffffL);
            if (d > dist[i]) {
                continue;
            }
            int ix = i % nx;
            int iy = i / nx;
            for (int k = 0; k < 8; k++) {
                int jx = ix + DX[k];
                int jy = iy + DY[k];
                if (jx < 0 || jy < 0 || jx >= nx || jy >= ny) {
                    continue;
                }
                int j = jy * nx + jx;
                if (block[j] != RasterMap.FREE) {
                    continue;
                }
                boolean diagonal = DX[k] != 0 && DY[k] != 0;
                if (diagonal) {
                    // Не срезаем угол между двумя заблокированными или занятыми ячейками.
                    int a = iy * nx + jx;
                    int b = jy * nx + ix;
                    if (block[a] != RasterMap.FREE || block[b] != RasterMap.FREE || occ[a] != 0 || occ[b] != 0) {
                        continue;
                    }
                }
                float nd = d + (float) ((diagonal ? cell * SQRT2 : cell) * (mult[i] + mult[j]) * 0.5);
                if (nd < dist[j]) {
                    dist[j] = nd;
                    prev[j] = i;
                    if (occ[j] == 0) {
                        push(nd, j);
                    }
                }
            }
        }
    }

    /** Стоимость пути до точки в «метрах с надбавками зон»; INF, если точка недостижима. */
    float costAt(double x, double y) {
        int idx = map.indexOf(x, y);
        return idx < 0 ? INF : dist[idx];
    }

    static boolean reachable(float cost) {
        return cost < INF;
    }

    /** Путь от стартовой точки до точки goal: сглаженная ломаная либо null, если цели не достичь. */
    double[][] path(double[] startXY, double[] goal, boolean smooth) {
        int g = map.indexOf(goal[0], goal[1]);
        if (g < 0 || dist[g] >= INF) {
            return null;
        }
        List<double[]> raw = new ArrayList<>();
        for (int i = g; i != -1 && i != start; i = prev[i]) {
            raw.add(new double[]{map.centerX(i), map.centerY(i)});
        }
        raw.add(startXY);
        java.util.Collections.reverse(raw);
        // Первая и последняя точки — настоящие (точка ОКС и точка врезки), а не центры ячеек.
        if (raw.size() == 1) {
            raw.add(goal);
        } else {
            raw.set(raw.size() - 1, goal);
        }
        double[][] pts = raw.toArray(new double[0][]);
        return smooth ? straighten(pts, goal) : pts;
    }

    /** Убирает лишние повороты: идём от точки к самой дальней точке пути, до которой есть свободная прямая. */
    private double[][] straighten(double[][] raw, double[] goal) {
        int n = raw.length;
        double[] prefix = new double[n];
        for (int k = 1; k < n; k++) {
            prefix[k] = prefix[k - 1] + Geo.dist(raw[k - 1], raw[k]) * pairMultiplier(raw[k - 1], raw[k]);
        }
        List<double[]> out = new ArrayList<>();
        out.add(raw[0]);
        int i = 0;
        while (i < n - 1) {
            int j = n - 1;
            for (; j > i + 1; j--) {
                double straight = lineCost(raw[i], raw[j], goal);
                if (straight >= 0 && straight <= prefix[j] - prefix[i] + 1e-6) {
                    break;
                }
            }
            out.add(raw[j]);
            i = j;
        }
        return out.toArray(new double[0][]);
    }

    private double pairMultiplier(double[] a, double[] b) {
        int ia = map.indexOf(a[0], a[1]);
        int ib = map.indexOf(b[0], b[1]);
        if (ia < 0 || ib < 0) {
            return 1;
        }
        return (mult[ia] + mult[ib]) * 0.5;
    }

    /**
     * Стоимость прямой a-b (с надбавками зон) или -1, если она задевает заблокированную или занятую ячейку.
     * Занятые ячейки рядом с целью разрешены: путь как раз заканчивается на существующей трубе.
     */
    private double lineCost(double[] a, double[] b, double[] goal) {
        double len = Geo.dist(a, b);
        double step = map.cell * 0.4;
        int n = Math.max(1, (int) Math.ceil(len / step));
        double cost = 0;
        double seg = len / n;
        for (int k = 0; k <= n; k++) {
            double t = (double) k / n;
            double x = a[0] + t * (b[0] - a[0]);
            double y = a[1] + t * (b[1] - a[1]);
            int idx = map.indexOf(x, y);
            if (idx < 0 || block[idx] != RasterMap.FREE) {
                return -1;
            }
            if (occ[idx] != 0 && Math.hypot(x - goal[0], y - goal[1]) > 2.5 * map.cell) {
                return -1;
            }
            if (k > 0) {
                cost += seg * mult[idx];
            }
        }
        return cost;
    }

    // ---- Куча из long: старшие 32 бита — стоимость (float), младшие — номер ячейки ---------------------------------

    private void push(float key, int idx) {
        if (heapSize == heap.length) {
            heap = Arrays.copyOf(heap, heap.length * 2);
        }
        long v = ((long) Float.floatToIntBits(key) << 32) | (idx & 0xffffffffL);
        int i = heapSize++;
        while (i > 0) {
            int p = (i - 1) >>> 1;
            if (heap[p] <= v) {
                break;
            }
            heap[i] = heap[p];
            i = p;
        }
        heap[i] = v;
    }

    private long pop() {
        long top = heap[0];
        long last = heap[--heapSize];
        int i = 0;
        while (true) {
            int c = 2 * i + 1;
            if (c >= heapSize) {
                break;
            }
            if (c + 1 < heapSize && heap[c + 1] < heap[c]) {
                c++;
            }
            if (heap[c] >= last) {
                break;
            }
            heap[i] = heap[c];
            i = c;
        }
        if (heapSize > 0) {
            heap[i] = last;
        }
        return top;
    }
}

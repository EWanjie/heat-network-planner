package ru.heatplanner;

/**
 * Перевод координат между WGS 84 (EPSG:4326, градусы) и WGS 84 / UTM zone 37N (EPSG:32637, метры).
 * По техническому приложению все длины, расстояния и буферы считаются именно в EPSG:32637.
 *
 * Формулы Крюгера (ряды по третьему порядку): точность на масштабе города — доли миллиметра.
 * Внешних библиотек не нужно.
 */
public final class Utm {

    private static final double A = 6378137.0;
    private static final double F = 1 / 298.257223563;
    private static final double K0 = 0.9996;
    private static final double E0 = 500000.0;
    /** Осевой меридиан зоны 37. */
    private static final double LON0 = Math.toRadians(39.0);

    private static final double N = F / (2 - F);
    private static final double E = Math.sqrt(F * (2 - F));
    private static final double AA = A / (1 + N) * (1 + N * N / 4 + Math.pow(N, 4) / 64);
    private static final double[] ALPHA = {
            N / 2 - 2 * N * N / 3 + 5 * Math.pow(N, 3) / 16,
            13 * N * N / 48 - 3 * Math.pow(N, 3) / 5,
            61 * Math.pow(N, 3) / 240};
    private static final double[] BETA = {
            N / 2 - 2 * N * N / 3 + 37 * Math.pow(N, 3) / 96,
            N * N / 48 + Math.pow(N, 3) / 15,
            17 * Math.pow(N, 3) / 480};
    private static final double[] DELTA = {
            2 * N - 2 * N * N / 3 - 2 * Math.pow(N, 3),
            7 * N * N / 3 - 8 * Math.pow(N, 3) / 5,
            56 * Math.pow(N, 3) / 15};

    private Utm() {
    }

    private static double atanh(double x) {
        return 0.5 * Math.log((1 + x) / (1 - x));
    }

    /** Градусы (долгота, широта) → метры {x (восток), y (север)}. */
    public static double[] forward(double lonDeg, double latDeg) {
        double phi = Math.toRadians(latDeg);
        double dl = Math.toRadians(lonDeg) - LON0;
        double t = Math.sinh(atanh(Math.sin(phi)) - E * atanh(E * Math.sin(phi)));
        double xi = Math.atan2(t, Math.cos(dl));
        double eta = atanh(Math.sin(dl) / Math.sqrt(1 + t * t));
        double x = eta;
        double y = xi;
        for (int j = 1; j <= 3; j++) {
            x += ALPHA[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
            y += ALPHA[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
        }
        return new double[]{E0 + K0 * AA * x, K0 * AA * y};
    }

    /** Метры {x, y} → градусы {долгота, широта}. */
    public static double[] inverse(double x, double y) {
        double xi = y / (K0 * AA);
        double eta = (x - E0) / (K0 * AA);
        double xiP = xi;
        double etaP = eta;
        for (int j = 1; j <= 3; j++) {
            xiP -= BETA[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
            etaP -= BETA[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
        }
        double chi = Math.asin(Math.sin(xiP) / Math.cosh(etaP));
        double phi = chi;
        for (int j = 1; j <= 3; j++) {
            phi += DELTA[j - 1] * Math.sin(2 * j * chi);
        }
        double lon = LON0 + Math.atan2(Math.sinh(etaP), Math.cos(xiP));
        return new double[]{Math.toDegrees(lon), Math.toDegrees(phi)};
    }
}

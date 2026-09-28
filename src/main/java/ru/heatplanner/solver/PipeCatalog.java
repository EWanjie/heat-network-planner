package ru.heatplanner.solver;

/** Competition 2D catalogue. Length/price refer to the pair, not each pipe. */
public final class PipeCatalog {
    private PipeCatalog() { }
    public static final int[] DN = {50,65,80,100,125,150,200,250,300,400,500,600,700,800,900,1000,1200,1400};
    public static final double[] CAPACITY = {3.5,8.3,13.2,22.3,40.2,65.1,152.3,274.9,437.4,943.1,1663.4,2627.7,3735.1,5296.8,7165,9391.8,15012.8,22501.9};
    public static final double[] MAX_LENGTH = {181,245,327,419,554,696,1042,1379,1718,2477,3245,4037,4775,5644,6518,7419,9288,11276};
    public static final double[] PRICE = {74023,78631,83530,89748,97275,105507,120275,135323,150022,190299,224137,264790,324298,325996,327693,418777,428074,683417};
    public static final double[] WIDTH = {.400,.430,.470,.510,.600,.650,.880,1.050,1.150,1.370,1.670,1.850,2.050,2.250,2.450,2.650,3.100,3.450};
    public static int index(int diameter) {
        for (int i=0;i<DN.length;i++) if (DN[i]==diameter) return i;
        throw new IllegalArgumentException("Неподдерживаемый ДУ: " + diameter);
    }
    public static int minimum(double flow, double length) {
        for(int i=0;i<DN.length;i++) if(CAPACITY[i]>=flow && MAX_LENGTH[i]>=length) return i;
        return -1;
    }
    public static double chamber(int index) {
        return DN[index]<=200 ? 3_000_000 : DN[index]<=500 ? 5_000_000 : DN[index]<=1000 ? 8_000_000 : 12_000_000;
    }
    public static double score(double cost, double length) { return .7*cost/25_000_000+.3*length/100; }
}

package ru.heatplanner.solver;

/** Search budgets are engineering choices, never competition constraints. */
public class SearchOptions {
    public int beamWidth = 4;
    public int initialTieInCandidateCount = 4;
    public int maxTieInCandidateCount = 16;
    public int neighborCount = 16;
    public int maxGraphRefinements = 3;
    public int maxRepairIterations = 5;
    public int maxLocalSearchIterations = 3;
    public int maxExpansionsPerRoute = 1500;
    public long totalSearchBudget = 250000;
    public long maxTimeMillis = 120000;
    public long seed = 17;
    public double tieInSamplingStep = 40;
    public double similarityToleranceMeters = 5;
    public double maxOverlapForDifferentCorridor = .85;
    public double maxAlternativeScoreDeterioration = .35;
    public void validate() {
        if (beamWidth<1 || beamWidth>32 || initialTieInCandidateCount<1 || maxTieInCandidateCount<initialTieInCandidateCount
                || maxTieInCandidateCount>128 || neighborCount<2 || neighborCount>256 || maxGraphRefinements<1 || maxGraphRefinements>6
                || maxRepairIterations<1 || maxRepairIterations>20 || maxLocalSearchIterations<0 || maxLocalSearchIterations>50
                || maxExpansionsPerRoute<1 || totalSearchBudget<1 || totalSearchBudget>10_000_000 || maxTimeMillis<1 || maxTimeMillis>600000
                || !Double.isFinite(tieInSamplingStep) || tieInSamplingStep<1 || !Double.isFinite(similarityToleranceMeters) || similarityToleranceMeters<=0
                || !Double.isFinite(maxOverlapForDifferentCorridor) || maxOverlapForDifferentCorridor<=0 || maxOverlapForDifferentCorridor>1
                || !Double.isFinite(maxAlternativeScoreDeterioration) || maxAlternativeScoreDeterioration<0)
            throw new IllegalArgumentException("Некорректные параметры поиска");
    }
}

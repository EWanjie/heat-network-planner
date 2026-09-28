package ru.heatplanner.solver;

import java.util.function.BooleanSupplier;

final class SearchBudget {
    final SearchOptions options;
    final BooleanSupplier cancelled;
    final long started=System.nanoTime();
    long expansions, transitions, routeCalls, candidates, validations, refinements;
    long scopeExpansions=Long.MAX_VALUE,scopeDeadline=Long.MAX_VALUE;
    SearchBudget(SearchOptions options, BooleanSupplier cancelled) {this.options=options; this.cancelled=cancelled;}
    boolean exhausted() {
        return cancelled.getAsBoolean() || Thread.currentThread().isInterrupted() || expansions>=options.totalSearchBudget || expansions>=scopeExpansions || System.nanoTime()>=scopeDeadline
                || (System.nanoTime()-started)/1_000_000>=options.maxTimeMillis;
    }
}

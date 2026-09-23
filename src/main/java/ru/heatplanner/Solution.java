package ru.heatplanner;

import java.util.ArrayList;
import java.util.List;

/** Готовый расчёт одного варианта: все объекты будущего выходного файла и стоимость по разделу 8. */
final class Solution {

    /** Технический узел: точка смены диаметра или способа прокладки внутри трубы. */
    static final class TechNode {
        final double[] xy;
        final String reason;

        TechNode(double[] xy, String reason) {
            this.xy = xy;
            this.reason = reason;
        }
    }

    /** Участок новой сети с одним набором параметров. start и end — Tree.Node или TechNode. */
    static final class Piece {
        Object start;
        Object end;
        double[][] pts;
        double length;
        double flow;
        int dn;
        boolean special;
        double k;
        String zoneLabel;
        double cost;
    }

    /** Новая камера (развилка или камера в точке врезки в участок). */
    static final class NewChamber {
        Tree.Node node;
        int dn;
        double cost;
    }

    static final class TieIn {
        Tree.Node node;
        String existingId;
        String existingType;
        int existingDn;
        int requiredDn;
        double cost = Rules.TIE_IN_COST;
    }

    static final class Recon {
        PlanModel.Segment segment;
        double[][] pts;
        double existingFlow;
        double addedFlow;
        int existingDn;
        int requiredDn;
        double length;
        double cost;
    }

    static final class ChamberRecon {
        PlanModel.Chamber chamber;
        int existingDn;
        int requiredDn;
        double cost;
    }

    /** Влияние на существующий участок: как меняется его загрузка. */
    static final class Impact {
        PlanModel.Segment segment;
        double addedFlow;
        double utilizationBefore;
        double utilizationAfter;
        int requiredDn;
    }

    final List<Piece> pieces = new ArrayList<>();
    final List<NewChamber> chambers = new ArrayList<>();
    final List<TieIn> tieIns = new ArrayList<>();
    final List<Recon> recons = new ArrayList<>();
    final List<ChamberRecon> chamberRecons = new ArrayList<>();
    final List<TechNode> techNodes = new ArrayList<>();
    final List<Impact> impacts = new ArrayList<>();
    final List<PlanModel.Target> unconnected = new ArrayList<>();

    double constructionCost;
    double chamberCost;
    double tieInCost;
    double reconCost;
    double chamberReconCost;
    double penalty;
    double newLength;
    double reconLength;
    /** true — расчёт невозможен (например, расход больше самого большого диаметра). */
    boolean invalid;

    double totalCost() {
        return constructionCost + chamberCost + tieInCost + reconCost + chamberReconCost + penalty;
    }

    double totalLength() {
        return newLength + reconLength;
    }

    double score() {
        return Rules.score(totalCost(), totalLength());
    }
}

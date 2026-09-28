package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.JsonNode;
import org.locationtech.jts.geom.*;
import java.util.*;
import static ru.heatplanner.solver.MetricGeometry.*;

/** Reads exported GeoJSON afresh, restores its graph and verifies reported aggregates. */
public final class ResultValidator {
    public List<String> validate(JsonNode input,JsonNode osm,JsonNode output) {
        List<String> errors=new ArrayList<>();
        try {validate(new PlanningContext(input,osm),output,errors);}catch(RuntimeException e){errors.add(e.getMessage()==null?e.getClass().getSimpleName():e.getMessage());}
        return errors;
    }
    List<String> validate(PlanningContext context,JsonNode output) {List<String>errors=new ArrayList<>();try{validate(context,output,errors);}catch(RuntimeException e){errors.add(e.getMessage()==null?"INVALID_OUTPUT":e.getMessage());}return errors;}
    private String key(JsonNode id){if(id==null||(!id.isTextual()&&!id.isNumber()))throw new IllegalArgumentException("INVALID_ID");return id.isTextual()?"s:"+id.asText():"n:"+id.decimalValue().stripTrailingZeros().toPlainString();}
    private void require(boolean condition,String error){if(!condition)throw new IllegalArgumentException(error);}
    private void close(double actual,double reported,String error){require(Double.isFinite(reported)&&Math.abs(actual-reported)<=Math.max(.02,Math.abs(actual)*1e-6),error);}
    private void validate(PlanningContext c,JsonNode output,List<String>errors) {
        NetworkCandidate n=new NetworkCandidate();Map<String,JsonNode> nodeProperties=new HashMap<>();
        Map<String,PlanningContext.Feature> originals=new HashMap<>();
        for(PlanningContext.Feature f:c.targets)originals.put(f.key,f);for(PlanningContext.Feature f:c.chambers)originals.put(f.key,f);
        JsonNode summary=null;Set<String> ids=new HashSet<>(c.ids);List<JsonNode> lineFeatures=new ArrayList<>();
        for(JsonNode f:output.path("features")) {
            JsonNode p=f.path("properties");String id=key(p.get("id"));require(ids.add(id),"DUPLICATE_ID");
            String type=p.path("object_type").asText();
            if(type.equals("variant_summary")){require(summary==null&&f.path("geometry").isNull(),"INVALID_SUMMARY");summary=p;continue;}
            if(type.equals("heat_network")){lineFeatures.add(f);continue;}
            require(type.equals("heat_chamber")||type.equals("technical_node"),"UNSUPPORTED_OUTPUT_OBJECT");
            Geometry g=c.metric.read(f.path("geometry"));require(g instanceof Point,"NODE_NOT_POINT");Coordinate point=g.getCoordinate();
            PlanningContext.Root root=null;
            if(p.has("existing_object_id")) {
                PlanningContext.Feature pipe=c.pipes.stream().filter(x->key(x.id).equals(key(p.get("existing_object_id")))).findFirst().orElse(null);
                require(type.equals("heat_chamber")&&pipe!=null&&pipe.geometry.distance(g)<.01,"INVALID_NEW_TIE_IN");root=new PlanningContext.Root(point,pipe,null);
            }
            n.nodes.put(id,new NetworkCandidate.Node(id,point,root,null));nodeProperties.put(id,p);
        }
        require(summary!=null,"MISSING_SUMMARY");
        Map<String,JsonNode> originalEdgeProperties=new HashMap<>();
        double reportedPipeCost=0,reportedLength=0,reportedCameras=0;
        for(JsonNode f:lineFeatures) {
            JsonNode p=f.path("properties");String from=key(p.get("start_node_id")),to=key(p.get("end_node_id"));
            for(String nodeId:List.of(from,to))if(!n.nodes.containsKey(nodeId)) {
                PlanningContext.Feature original=originals.get(nodeId);require(original!=null,"MISSING_NODE_REFERENCE");
                PlanningContext.Root root=null;
                if(original.type.equals("heat_chamber")) {
                    PlanningContext.Feature pipe=c.pipes.stream().filter(x->x.geometry.distance(original.geometry)<.01).findFirst().orElse(null);
                    require(pipe!=null,"CAMERA_NOT_ON_NETWORK");root=new PlanningContext.Root(original.geometry.getCoordinate(),pipe,original);
                }
                n.nodes.put(nodeId,new NetworkCandidate.Node(nodeId,original.geometry.getCoordinate(),root,root==null?original:null));
            }
            Geometry geometry=c.metric.read(f.path("geometry"));require(geometry instanceof LineString,"PIPE_NOT_LINE");
            int d=PipeCatalog.index(p.path("diameter").asInt());
            NetworkCandidate.Edge edge=new NetworkCandidate.Edge(from,to,(LineString)geometry,d);edge.flow=p.path("flow_tph").asDouble(Double.NaN);
            require(Double.isFinite(edge.flow)&&edge.flow>0,"INVALID_FLOW");
            require(p.has("depth_start")&&p.get("depth_start").isNull()&&p.has("depth_end")&&p.get("depth_end").isNull(),"DEPTH_NOT_NULL");
            close(geometry.getLength(),p.path("length").asDouble(Double.NaN),"WRONG_LENGTH");
            double k=p.path("special_coefficient").asDouble(1);require(Double.isFinite(k)&&k>=1,"INVALID_COEFFICIENT");
            close(geometry.getLength()*PipeCatalog.PRICE[d]*k,p.path("cost").asDouble(Double.NaN),"WRONG_PIPE_PRICE");
            require(p.path("laying_method").asText().equals(k>1?"special":"base"),"WRONG_LAYING_METHOD");
            require(originalEdgeProperties.put(edge.key(),p)==null,"DUPLICATE_EDGE");
            reportedPipeCost+=p.path("cost").asDouble();reportedLength+=p.path("length").asDouble();n.edges.add(edge);
        }
        // Check declared special intervals before removing tariff-only technical vertices.
        Map<String,List<NetworkCandidate.Edge>> fragments=new HashMap<>();for(NetworkCandidate.Edge e:n.edges)fragments.put(e.key(),new ArrayList<>(List.of(e.copy())));
        boolean changed=true;
        while(changed) {
            changed=false;
            for(NetworkCandidate.Node node:new ArrayList<>(n.nodes.values()))if(node.root==null&&node.target==null&&n.children(node.key).size()==1) {
                NetworkCandidate.Edge in=n.incoming(node.key);if(in==null)continue;NetworkCandidate.Edge out=n.children(node.key).get(0);
                require(in.diameter==out.diameter&&Math.abs(in.flow-out.flow)<1e-6,"CONSTANT_FLOW_CHAIN_CHANGED");
                require(nodeProperties.get(node.key).path("object_type").asText().equals("technical_node"),"UNNECESSARY_CHAMBER");
                List<Coordinate> coordinates=new ArrayList<>(Arrays.asList(in.route.getCoordinates()));coordinates.addAll(Arrays.asList(out.route.getCoordinates()).subList(1,out.route.getNumPoints()));
                NetworkCandidate.Edge merged=new NetworkCandidate.Edge(in.from,out.to,line(coordinates.toArray(new Coordinate[0])),in.diameter);merged.flow=in.flow;
                List<NetworkCandidate.Edge> parts=new ArrayList<>(fragments.remove(in.key()));parts.addAll(fragments.remove(out.key()));fragments.put(merged.key(),parts);
                n.edges.remove(in);n.edges.remove(out);n.edges.add(merged);n.nodes.remove(node.key);changed=true;break;
            }
        }
        for(JsonNode id:summary.path("unconnected_oks_ids"))require(n.pending.add(key(id)),"DUPLICATE_PENDING_TARGET");
        for(String pending:n.pending)require(c.targets.stream().anyMatch(t->t.key.equals(pending)),"UNKNOWN_PENDING_TARGET");
        GeometryRules rules=new GeometryRules(c);
        for(NetworkCandidate.Edge edge:n.edges) {
            GeometryRules.Check check=rules.check(edge.route,edge.diameter,n.nodes.get(edge.to).target,n.nodes.get(edge.from).root);
            require(check.valid,"INVALID_GEOMETRY:"+check.reason);
            double offset=0;
            for(NetworkCandidate.Edge part:fragments.get(edge.key())) {
                double end=offset+part.route.getLength(),weighted=0;
                for(GeometryRules.Interval interval:GeometryRules.costIntervals(edge.route.getLength(),check.intervals))
                    weighted+=Math.max(0,Math.min(end,interval.to)-Math.max(offset,interval.from))*interval.k;
                JsonNode p=originalEdgeProperties.get(part.key());close(weighted*PipeCatalog.PRICE[part.diameter],p.path("cost").asDouble(Double.NaN),"WRONG_SPECIAL_INTERVAL_PRICE");offset=end;
            }
        }
        NetworkValidator validator=new NetworkValidator(c);require(validator.validate(n),"INVALID_NETWORK:"+validator.failure);
        for(NetworkCandidate.Node node:n.nodes.values())if(nodeProperties.containsKey(node.key)) {
            JsonNode p=nodeProperties.get(node.key);boolean chamber=node.root!=null||n.children(node.key).size()>1;
            require(p.path("object_type").asText().equals(chamber?"heat_chamber":"technical_node"),"MISSING_BRANCH_CHAMBER");
            if(chamber) {
                int d=node.root==null?0:c.existingDiameter(node.point);for(NetworkCandidate.Edge e:n.edges)if(e.from.equals(node.key)||e.to.equals(node.key))d=Math.max(d,e.diameter);
                require(p.path("diameter").asInt()==PipeCatalog.DN[d],"WRONG_CHAMBER_DIAMETER");close(PipeCatalog.chamber(d),p.path("cost").asDouble(Double.NaN),"WRONG_CHAMBER_PRICE");reportedCameras+=p.path("cost").asDouble();
            }
        }
        close(n.chamberCost,reportedCameras,"CHAMBER_TOTAL");close(n.length,reportedLength,"LENGTH_TOTAL");
        close(n.cost,reportedPipeCost+reportedCameras+n.tieInCost,"COST_TOTAL");
        close(n.cost,summary.path("construction_cost").asDouble(Double.NaN),"CONSTRUCTION_COST");
        close(n.chamberCost,summary.path("chamber_construction_cost").asDouble(Double.NaN),"CHAMBER_COST");
        require(n.tieIns==summary.path("existing_chamber_tie_in_count").asInt(-1),"TIE_IN_COUNT");
        close(n.tieInCost,summary.path("existing_chamber_tie_in_cost").asDouble(Double.NaN),"TIE_IN_COST");
        close(n.penalty,summary.path("unconnected_penalty").asDouble(Double.NaN),"PENALTY");
        close(n.cost+n.penalty,summary.path("calculated_cost").asDouble(Double.NaN),"CALCULATED_COST");
        close(n.length,summary.path("new_network_length").asDouble(Double.NaN),"NETWORK_LENGTH");
        require(Math.abs(n.score-summary.path("score").asDouble(Double.NaN))<1e-5,"SCORE");
    }
}

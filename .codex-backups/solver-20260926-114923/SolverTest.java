package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.*;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;
import static ru.heatplanner.solver.MetricGeometry.*;

class SolverTest {
    final ObjectMapper mapper=new ObjectMapper();final MetricGeometry metric=new MetricGeometry();
    // Real UTM coordinates, so production projection is exercised even in synthetic fixtures.
    Coordinate p(double x,double y){return new Coordinate(414000+x,6179000+y);}
    ObjectNode feature(Object id,String type,Geometry geometry) {
        ObjectNode f=mapper.createObjectNode();f.put("type","Feature");f.set("geometry",metric.write(geometry,mapper));
        ObjectNode props=f.putObject("properties");props.set("id",mapper.valueToTree(id));props.put("object_type",type);return f;
    }
    ObjectNode target(Object id,double x,double y,double flow){ObjectNode f=feature(id,"oks_connection_point",point(p(x,y)));((ObjectNode)f.get("properties")).put("flow_tph",flow);return f;}
    ObjectNode pipe(){ObjectNode f=feature("pipe","heat_network",line(p(-10,0),p(10,0)));((ObjectNode)f.get("properties")).put("diameter",100);return f;}
    ObjectNode restriction(String id,String type,Geometry g){ObjectNode f=feature(id,"restriction",g);((ObjectNode)f.get("properties")).put("restriction_type",type);return f;}
    Polygon box(double x1,double y1,double x2,double y2){return GF.createPolygon(new Coordinate[]{p(x1,y1),p(x2,y1),p(x2,y2),p(x1,y2),p(x1,y1)});}
    ObjectNode collection(JsonNode... features){ObjectNode c=mapper.createObjectNode();c.put("type","FeatureCollection");ArrayNode a=c.putArray("features");for(JsonNode f:features)a.add(f);return c;}
    ObjectNode basic(){return collection(pipe(),feature("camera","heat_chamber",point(p(0,0))),target("goal",0,100,20));}
    SearchOptions fast(){SearchOptions o=new SearchOptions();o.beamWidth=2;o.maxLocalSearchIterations=0;o.maxTimeMillis=20000;o.totalSearchBudget=10000;o.initialTieInCandidateCount=2;o.maxTieInCandidateCount=4;o.maxGraphRefinements=2;return o;}
    JsonNode solve(ObjectNode input){return mapper.valueToTree(new NetworkPlanner(mapper).solveNetwork(input,null,fast(),()->false));}
    JsonNode summary(JsonNode variant){for(JsonNode f:variant.path("features"))if(f.path("properties").path("object_type").asText().equals("variant_summary"))return f.get("properties");throw new AssertionError("No summary");}
    @Test void catalogueControlCaseAndGeoJsonContract(){
        JsonNode result=solve(basic());assertFalse(result.path("variants").isEmpty(),result.toPrettyString());
        JsonNode v=result.path("variants").get(0),s=summary(v);
        assertEquals(100,s.path("new_network_length").asDouble(),.001);
        assertEquals(13_974_800,s.path("construction_cost").asDouble(),10);
        assertEquals(.6912944,s.path("score").asDouble(),.000001);
        assertEquals(1,s.path("existing_chamber_tie_in_count").asInt());
        assertTrue(s.path("unconnected_oks_ids").isEmpty());
        Set<JsonNode> ids=new HashSet<>();for(JsonNode f:v.path("features"))assertTrue(ids.add(f.path("properties").get("id")));
        for(JsonNode f:v.path("features"))if(f.path("properties").path("object_type").asText().equals("heat_network")) {
            assertEquals(100,f.path("properties").path("diameter").asInt());assertTrue(f.path("properties").get("depth_start").isNull());
        }
    }
    @Test void roadPolygonIntervalsAndAngle(){
        PlanningContext c=new PlanningContext(collection(restriction("road","road",box(-50,40,50,60))),null);GeometryRules rules=new GeometryRules(c);
        GeometryRules.Check check=rules.check(line(p(0,0),p(0,100)),3,null,null);
        assertTrue(check.valid,check.reason);assertEquals(115.6,check.weightedLength,.001);
        assertFalse(rules.check(line(p(-80,20),p(80,70)),3,null,null).valid);
        assertFalse(rules.check(line(p(0,0),p(0,50),p(10,100)),3,null,null).valid);
    }
    @Test void missingRoadWidthCannotBecomeFreeCrossing(){
        PlanningContext c=new PlanningContext(collection(restriction("road","road",line(p(-100,50),p(100,50)))),null);
        GeometryRules.Check check=new GeometryRules(c).check(line(p(0,0),p(0,100)),3,null,null);
        assertFalse(check.valid);assertEquals("ROAD_WIDTH_REQUIRED",check.reason);
        assertTrue(c.diagnostics.stream().anyMatch(d->d.get("code").equals("ROAD_WIDTH_REQUIRED")));
    }
    @Test void validOsmWidthCreatesMetricArea(){
        ObjectNode f=restriction("road","road",line(p(-100,50),p(100,50)));
        ((ObjectNode)f.get("properties")).put("width_m",10).put("width_source","osm_width_tag");
        PlanningContext c=new PlanningContext(collection(f),null);
        assertEquals(2,c.obstacles.get(0).geometry.getDimension());
        GeometryRules.Check check=new GeometryRules(c).check(line(p(0,0),p(0,100)),3,null,null);
        assertTrue(check.valid,check.reason);assertEquals(109.6,check.weightedLength,.001);
    }
    @Test void hardObstacleAndHoleAreNotFilledOrIgnored(){
        LinearRing shell=GF.createLinearRing(box(-100,-100,100,100).getCoordinates());
        LinearRing hole=GF.createLinearRing(box(-20,-20,20,20).getCoordinates());
        PlanningContext c=new PlanningContext(collection(restriction("water","water",GF.createPolygon(shell,new LinearRing[]{hole}))),null);
        GeometryRules rules=new GeometryRules(c);
        assertTrue(rules.check(line(p(-5,0),p(5,0)),0,null,null).valid);
        assertFalse(rules.check(line(p(0,0),p(150,0)),0,null,null).valid);
    }
    @Test void ownBuildingApproachKeepsOtherRestrictions(){
        PlanningContext c=new PlanningContext(collection(target("goal",0,100,20),restriction("own","oks",box(-10,90,10,110))),null);
        assertTrue(new GeometryRules(c).check(line(p(0,0),p(0,100)),3,c.targets.get(0),null).valid);
        assertFalse(new GeometryRules(c).check(line(p(0,0),p(0,100)),3,null,null).valid);
        PlanningContext blocked=new PlanningContext(collection(target("goal",0,100,20),restriction("own","oks",box(-10,90,10,110)),restriction("water","water",box(-2,93,2,95))),null);
        assertFalse(new GeometryRules(blocked).check(line(p(0,0),p(0,100)),3,blocked.targets.get(0),null).valid);
    }
    @Test void fixedSpecialClearanceConflictIsExplicit(){
        PlanningContext c=new PlanningContext(collection(restriction("gas","gas_pipeline",line(p(-10,50),p(10,50)))),null);
        GeometryRules.Check check=new GeometryRules(c).check(line(p(0,0),p(0,100)),3,null,null);
        assertFalse(check.valid);assertEquals("RULE_CONFLICT",check.reason);
    }
    @Test void idsRetainNumericVersusStringIdentity(){
        PlanningContext c=new PlanningContext(collection(target(1,0,100,1),target("1",10,100,1)),null);assertEquals(2,c.targets.size());
        assertNotEquals(c.targets.get(0).key,c.targets.get(1).key);
        assertThrows(IllegalArgumentException.class,()->new PlanningContext(collection(target("same",0,100,1),target("same",10,100,1)),null));
    }
    @Test void cancellationReturnsNoFabricatedVariant(){
        JsonNode result=mapper.valueToTree(new NetworkPlanner(mapper).solveNetwork(basic(),null,fast(),()->true));
        assertTrue(result.path("variants").isEmpty());assertTrue(result.path("searchStatistics").path("budgetExhausted").asBoolean());
    }
    @Test void visibilityGraphActuallyDetoursAroundBuilding(){
        PlanningContext c=new PlanningContext(collection(restriction("block","oks",box(-10,40,10,60))),null);
        SearchBudget budget=new SearchBudget(fast(),()->false);
        LineString route=new RouteSearch(c,budget).find(p(0,0),p(0,100),3,419,null,null,RouteSearch.Profile.BALANCED,null);
        assertNotNull(route);assertTrue(route.getLength()>100);assertTrue(new GeometryRules(c).check(route,3,null,null).valid);
    }
    @Test void diameterDpKeepsLongestPathNotSumOfParallelBranches(){
        PlanningContext c=new PlanningContext(collection(),null);NetworkCandidate n=new NetworkCandidate();
        NetworkCandidate.Edge trunk=new NetworkCandidate.Edge("r","j",line(p(0,0),p(0,150)),0);trunk.flow=2;trunk.weightedLength=150;
        NetworkCandidate.Edge a=new NetworkCandidate.Edge("j","a",line(p(0,150),p(-100,150)),0);a.flow=1;a.weightedLength=100;
        NetworkCandidate.Edge b=new NetworkCandidate.Edge("j","b",line(p(0,150),p(100,150)),0);b.flow=1;b.weightedLength=100;
        n.edges.addAll(List.of(trunk,a,b));
        List<DiameterAssignment.State> states=new DiameterAssignment(c).states(n,trunk);
        assertFalse(states.isEmpty());
        // DN50 cannot continue through 250m. DN80 can (327m); parallel 100m branches are not summed.
        assertTrue(states.stream().anyMatch(s->s.diameter==2));
        for(DiameterAssignment.State s:states)assertTrue(s.suffix<=PipeCatalog.MAX_LENGTH[s.diameter]);
    }
    @Test void reorderingAndReversingInputDoesNotChangeControlCost(){
        ObjectNode original=basic(),reordered=basic();ArrayNode fs=(ArrayNode)reordered.get("features");
        JsonNode first=fs.remove(0);fs.add(first);
        ArrayNode coords=(ArrayNode)first.path("geometry").get("coordinates");JsonNode a=coords.remove(0);coords.add(a);
        assertEquals(summary(solve(original).path("variants").get(0)).path("score").asDouble(),summary(solve(reordered).path("variants").get(0)).path("score").asDouble(),1e-8);
    }
    @Test void twoTargetsCanShareAStemAndBeatIndependentBaseline(){
        ObjectNode input=collection(pipe(),feature("camera","heat_chamber",point(p(0,0))),target("a",-10,120,20),target("b",10,120,20));
        JsonNode result=solve(input);assertFalse(result.path("variants").isEmpty(),result.toPrettyString());
        JsonNode best=summary(result.path("variants").get(0));
        assertTrue(best.path("unconnected_oks_ids").isEmpty(),result.toPrettyString());
        double independent=2*(Math.hypot(10,120)*89748+5_000_000);
        assertTrue(best.path("construction_cost").asDouble()<independent,result.toPrettyString());
        assertTrue(best.path("new_network_length").asDouble()<2*Math.hypot(10,120));
    }
    @Test void osmFailureIsNotAnEmptySuccessfulRoadLayer(){
        ObjectNode meta=mapper.createObjectNode().put("required",true).put("status","error");
        JsonNode result=mapper.valueToTree(new NetworkPlanner(mapper).solveNetwork(basic(),meta,fast(),()->false));
        assertTrue(result.path("variants").isEmpty());
        assertTrue(result.path("diagnostics").toString().contains("OSM_DATA_UNAVAILABLE"));
    }
    @Test void exportedValidatorRejectsTamperedPriceFlowAndGeometry(){
        ObjectNode input=basic();JsonNode output=solve(input).path("variants").get(0);
        ResultValidator validator=new ResultValidator();assertTrue(validator.validate(input,null,output).isEmpty());
        ObjectNode changed=output.deepCopy();ObjectNode s=(ObjectNode)summary(changed);s.put("construction_cost",0);
        assertFalse(validator.validate(input,null,changed).isEmpty());
        changed=output.deepCopy();for(JsonNode f:changed.path("features"))if(f.path("properties").path("object_type").asText().equals("heat_network"))((ObjectNode)f.get("properties")).put("flow_tph",1);
        assertFalse(validator.validate(input,null,changed).isEmpty());
        changed=output.deepCopy();for(JsonNode f:changed.path("features"))if(f.path("properties").path("object_type").asText().equals("heat_network"))((ObjectNode)f.get("geometry")).set("coordinates",mapper.valueToTree(new double[][]{{37,55},{38,55}}));
        assertFalse(validator.validate(input,null,changed).isEmpty());
    }
}

#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <random>

namespace p = pongbot_local_graph_insertion_planner;
namespace {
p::GridSnapshot grid(std::size_t w, std::size_t h) {
  p::GridSnapshot g; g.frame_id="map";g.size_x=w;g.size_y=h;g.resolution=1.0;g.costs.assign(w*h,0);return g;
}
void expectSame(const p::GridSnapshot & g,std::size_t s,std::size_t t,p::DStarLite & d,const p::SearchOptions & o={}) {
  const auto a=p::freshAstar(g,s,t,o), b=d.replan(g,s,t,o);
  EXPECT_EQ(a.status==p::SearchStatus::kSuccess,b.status==p::SearchStatus::kSuccess);
  if(a.status==p::SearchStatus::kSuccess){EXPECT_TRUE(p::validPath(g,b.path,o));EXPECT_NEAR(a.cost,b.cost,1e-9);}
}
}
TEST(Core, EmptyAndStartEqualsGoal) {
  auto g=grid(5,5);p::DStarLite d;expectSame(g,0,24,d);auto r=d.replan(g,4,4,{});
  ASSERT_EQ(r.status,p::SearchStatus::kSuccess);ASSERT_EQ(r.path.size(),1u);EXPECT_EQ(r.cost,0.0);
}
TEST(Core, BoundsAndOccupiedEndpoints) {
  auto g=grid(3,3);EXPECT_EQ(p::freshAstar(g,9,0,{}).status,p::SearchStatus::kInvalidInput);
  g.costs[0]=253;EXPECT_EQ(p::freshAstar(g,0,8,{}).status,p::SearchStatus::kNoPath);
  g.costs[0]=0;g.costs[8]=254; p::DStarLite d;EXPECT_EQ(d.replan(g,0,8,{}).status,p::SearchStatus::kNoPath);
}
TEST(Core, UnknownAndCostBoundaryContract) {
  auto g=grid(3,1);g.costs[1]=255;EXPECT_EQ(p::freshAstar(g,0,2,{}).status,p::SearchStatus::kNoPath);
  p::SearchOptions known;known.allow_unknown=true;EXPECT_EQ(p::freshAstar(g,0,2,known).status,p::SearchStatus::kSuccess);
  g.costs[1]=252;EXPECT_EQ(p::freshAstar(g,0,2,{}).status,p::SearchStatus::kSuccess);
  g.costs[1]=253;EXPECT_EQ(p::freshAstar(g,0,2,{}).status,p::SearchStatus::kNoPath);
}
TEST(Core, DiagonalCannotCutCorners) {
  auto g=grid(2,2);g.costs[1]=253;g.costs[2]=253;
  EXPECT_EQ(p::freshAstar(g,0,3,{}).status,p::SearchStatus::kNoPath);
}
TEST(Core, StaticDStarMatchesFresh) {
  auto g=grid(15,15);for(std::size_t y=1;y<14;++y)if(y!=8)g.costs[g.index(7,y)]=253;
  p::DStarLite d;expectSame(g,0,224,d);
}
TEST(Core, ObstacleInsertionRemovalAndMultipleChanges) {
  auto g=grid(10,5);p::DStarLite d;expectSame(g,20,29,d);
  g.costs[g.index(4,2)]=253;expectSame(g,20,29,d);
  g.costs[g.index(4,2)]=0;expectSame(g,20,29,d);
  for(std::size_t x=2;x<7;++x) {
    g.costs[g.index(x,2)]=253;
  }
  expectSame(g,20,29,d);
}
TEST(Core, MovingStartAndGoalReset) {
  auto g=grid(10,10);p::DStarLite d;expectSame(g,0,99,d);expectSame(g,1,99,d);
  expectSame(g,1,98,d);EXPECT_EQ(d.lastFallbackReason(),p::FallbackReason::kGoalChanged);
}
TEST(Core, ChangedRatioUsesFreshFallback) {
  auto g=grid(10,10);p::DStarLite d(0.10);expectSame(g,0,99,d);
  for(std::size_t i=1;i<=20;++i)g.costs[i]=253;
  expectSame(g,0,99,d);EXPECT_EQ(d.lastFallbackReason(),p::FallbackReason::kChangedRatio);
}
TEST(Core, TimeoutAndCancellation) {
  auto g=grid(30,30);p::SearchOptions timeout;timeout.max_expansions=1;
  EXPECT_EQ(p::freshAstar(g,0,899,timeout).status,p::SearchStatus::kTimeout);
  p::DStarLite d;EXPECT_EQ(d.replan(g,0,899,timeout).status,p::SearchStatus::kTimeout);
  p::SearchOptions cancel;cancel.cancelled=[] {return true;};
  EXPECT_EQ(p::freshAstar(g,0,899,cancel).status,p::SearchStatus::kCancelled);
}
TEST(Core, InvalidFiniteContract) {
  auto g=grid(2,2);g.resolution=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(p::freshAstar(g,0,3,{}).status,p::SearchStatus::kInvalidInput);
}
TEST(Core, RandomizedDifferential) {
  std::mt19937 rng(20260723);std::bernoulli_distribution blocked(0.22);
  constexpr int kIterations=250;
  for(int iteration=0;iteration<kIterations;++iteration) {
    auto g=grid(18,17);for(auto & c:g.costs)if(blocked(rng))c=253;g.costs[0]=0;g.costs.back()=0;
    p::SearchOptions o;o.cost_penalty=(iteration%3)*0.35;p::DStarLite d;
    const auto a=p::freshAstar(g,0,g.costs.size()-1,o),b=d.replan(g,0,g.costs.size()-1,o);
    ASSERT_EQ(a.status==p::SearchStatus::kSuccess,b.status==p::SearchStatus::kSuccess) << iteration;
    if(a.status==p::SearchStatus::kSuccess){double cost=0;ASSERT_TRUE(p::validPath(g,b.path,o,&cost))<<iteration;ASSERT_NEAR(a.cost,b.cost,1e-9)<<iteration;}
  }
}

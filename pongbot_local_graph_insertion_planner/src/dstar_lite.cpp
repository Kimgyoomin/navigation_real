#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pongbot_local_graph_insertion_planner {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-10;
bool equal(double a, double b) { return std::abs(a - b) <= kEpsilon; }
}

const char * fallbackReasonName(FallbackReason reason) {
  switch (reason) {
    case FallbackReason::kNone: return "none"; case FallbackReason::kInitialPlan: return "initial_plan";
    case FallbackReason::kGeometryChanged: return "geometry_changed"; case FallbackReason::kGoalChanged: return "goal_changed";
    case FallbackReason::kOptionsChanged: return "options_changed"; case FallbackReason::kChangedRatio: return "changed_ratio";
    case FallbackReason::kInvariantViolation: return "invariant_violation"; case FallbackReason::kExtractionFailure: return "extraction_failure";
  } return "unknown";
}
DStarLite::DStarLite(double ratio) : changed_ratio_fallback_(ratio) {}
bool DStarLite::EntryCompare::operator()(const Entry & a, const Entry & b) const {
  return a.key.first == b.key.first ? a.key.second > b.key.second : a.key.first > b.key.first;
}
bool DStarLite::keyLess(const Key & a, const Key & b) const {
  return a.first + kEpsilon < b.first || (equal(a.first,b.first) && a.second + kEpsilon < b.second);
}
bool DStarLite::optionsEqual(const SearchOptions & o) const {
  return options_.allow_unknown == o.allow_unknown && options_.blocked_cost_threshold == o.blocked_cost_threshold &&
    options_.cost_penalty == o.cost_penalty;
}
bool DStarLite::cancelled() const { return options_.cancelled && options_.cancelled(); }
void DStarLite::reset(const GridSnapshot & grid, std::size_t start, std::size_t goal, const SearchOptions & o) {
  grid_=grid; options_=o; start_=last_start_=start; goal_=goal; km_=0.0; g_.assign(grid.costs.size(),kInfinity);
  rhs_.assign(grid.costs.size(),kInfinity); open_={}; rhs_[goal_]=0.0; push(goal_); initialized_=true;
}
DStarLite::Key DStarLite::calculateKey(std::size_t cell) const {
  const double best=std::min(g_[cell],rhs_[cell]); return {best+heuristic(grid_,start_,cell)+km_,best};
}
void DStarLite::push(std::size_t cell) { if (!equal(g_[cell],rhs_[cell])) open_.push({calculateKey(cell),cell}); }
void DStarLite::updateVertex(std::size_t cell) {
  if (cell != goal_) {
    double value=kInfinity;
    if (!isBlocked(grid_,cell,options_))
      for (auto next:neighbors8(grid_,cell,options_)) value=std::min(value,edgeCost(grid_,cell,next,options_)+g_[next]);
    rhs_[cell]=value;
  } push(cell);
}
void DStarLite::applyChanges(const GridSnapshot & grid) {
  std::vector<std::size_t> affected;
  for (std::size_t cell=0;cell<grid.costs.size();++cell) if (grid.costs[cell]!=grid_.costs[cell]) {
    const auto cx=grid.x(cell),cy=grid.y(cell);
    // Changed cell, its incident edges, and diagonal no-corner-cutting dependencies.
    for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx) {
      const auto x=static_cast<long long>(cx)+dx,y=static_cast<long long>(cy)+dy;
      if(x>=0&&y>=0&&x<static_cast<long long>(grid.size_x)&&y<static_cast<long long>(grid.size_y))
        affected.push_back(grid.index(static_cast<std::size_t>(x),static_cast<std::size_t>(y)));
    }
  }
  grid_=grid; std::sort(affected.begin(),affected.end()); affected.erase(std::unique(affected.begin(),affected.end()),affected.end());
  for(auto cell:affected) updateVertex(cell);
}
bool DStarLite::computeShortestPath(SearchResult & result) {
  while(!open_.empty()) {
    if(cancelled()){result.status=SearchStatus::kCancelled;result.reason="cancelled";return false;}
    if(options_.max_expansions&&result.expanded>=options_.max_expansions){result.status=SearchStatus::kTimeout;result.reason="expansion limit";return false;}
    const auto entry=open_.top();open_.pop();const auto current=calculateKey(entry.cell);
    if(keyLess(entry.key,current)){open_.push({current,entry.cell});continue;} if(keyLess(current,entry.key)||equal(g_[entry.cell],rhs_[entry.cell]))continue;
    ++result.expanded;
    if(g_[entry.cell]>rhs_[entry.cell]){g_[entry.cell]=rhs_[entry.cell];for(auto p:neighbors8(grid_,entry.cell,options_))updateVertex(p);}
    else {g_[entry.cell]=kInfinity;updateVertex(entry.cell);for(auto p:neighbors8(grid_,entry.cell,options_))updateVertex(p);}
    if((open_.empty()||!keyLess(open_.top().key,calculateKey(start_)))&&equal(g_[start_],rhs_[start_]))return true;
  } return equal(g_[start_],rhs_[start_]);
}
SearchResult DStarLite::extractPath() {
  SearchResult r;if(!std::isfinite(g_[start_])){r.status=SearchStatus::kNoPath;r.reason="start has infinite value";return r;}
  std::vector<bool> seen(grid_.costs.size(),false);auto current=start_;r.path.push_back(current);
  while(current!=goal_) {
    if(seen[current]){r.status=SearchStatus::kNoPath;r.reason="path extraction cycle";return r;}seen[current]=true;
    double best=kInfinity;std::size_t next=grid_.costs.size();
    for(auto s:neighbors8(grid_,current,options_)){const auto candidate=edgeCost(grid_,current,s,options_)+g_[s];if(candidate+kEpsilon<best){best=candidate;next=s;}}
    if(next>=grid_.costs.size()||!std::isfinite(best)){r.status=SearchStatus::kNoPath;r.reason="path extraction dead end";return r;}
    current=next;r.path.push_back(current);if(r.path.size()>grid_.costs.size()){r.status=SearchStatus::kNoPath;r.reason="path extraction length invariant";return r;}
  }
  r.status=SearchStatus::kSuccess;r.cost=g_[start_];double verified=0.0;
  if(!validPath(grid_,r.path,options_,&verified)||!equal(verified,r.cost)){r.status=SearchStatus::kNoPath;r.reason="path extraction cost invariant";r.path.clear();}return r;
}
SearchResult DStarLite::replan(const GridSnapshot & grid,std::size_t start,std::size_t goal,const SearchOptions & o) {
  last_fallback_reason_=FallbackReason::kNone;last_fallback_detail_.clear();
  if(!grid.valid()||!grid.validIndex(start)||!grid.validIndex(goal)||!std::isfinite(o.cost_penalty)||o.cost_penalty<0.0||!o.blocked_cost_threshold)
    return {SearchStatus::kInvalidInput, "invalid grid, options, or endpoint", {}, 0, 0.0};
  if(isBlocked(grid,start,o)||isBlocked(grid,goal,o))
    return {SearchStatus::kNoPath, "start or goal is blocked", {}, 0, 0.0};
  if(!initialized_||!grid.geometryEquals(grid_)){last_fallback_reason_=initialized_?FallbackReason::kGeometryChanged:FallbackReason::kInitialPlan;reset(grid,start,goal,o);}
  else if(goal!=goal_){last_fallback_reason_=FallbackReason::kGoalChanged;reset(grid,start,goal,o);}
  else if(!optionsEqual(o)){last_fallback_reason_=FallbackReason::kOptionsChanged;reset(grid,start,goal,o);}
  else {
    std::size_t changes=0;for(std::size_t i=0;i<grid.costs.size();++i)changes+=grid.costs[i]!=grid_.costs[i];
    if(static_cast<double>(changes)/grid.costs.size()>changed_ratio_fallback_){last_fallback_reason_=FallbackReason::kChangedRatio;last_fallback_detail_="fresh A* selected because changed-cell ratio exceeded threshold";reset(grid,start,goal,o);return freshAstar(grid,start,goal,o);}
    km_+=heuristic(grid_,last_start_,start);start_=start;last_start_=start;applyChanges(grid);
  }
  SearchResult work;if(!computeShortestPath(work))return work;auto r=extractPath();r.expanded=work.expanded;
  if(r.status!=SearchStatus::kSuccess){last_fallback_reason_=FallbackReason::kExtractionFailure;last_fallback_detail_=r.reason;return freshAstar(grid_,start_,goal_,options_);}return r;
}
}  // namespace pongbot_local_graph_insertion_planner

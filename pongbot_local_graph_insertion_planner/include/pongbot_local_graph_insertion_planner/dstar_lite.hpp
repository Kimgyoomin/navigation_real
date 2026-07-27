#pragma once

#include "pongbot_local_graph_insertion_planner/fresh_astar.hpp"

#include <cstddef>
#include <queue>
#include <string>
#include <vector>

namespace pongbot_local_graph_insertion_planner
{

enum class FallbackReason
{
  kNone,
  kInitialPlan,
  kGeometryChanged,
  kGoalChanged,
  kOptionsChanged,
  kChangedRatio,
  kInvariantViolation,
  kExtractionFailure
};

class DStarLite
{
public:
  explicit DStarLite(double changed_ratio_fallback = 0.35);
  SearchResult replan(
    const GridSnapshot & grid, std::size_t start, std::size_t goal, const SearchOptions & options);
  FallbackReason lastFallbackReason() const {return last_fallback_reason_;}
  const std::string & lastFallbackDetail() const {return last_fallback_detail_;}
  bool initialized() const {return initialized_;}

private:
  struct Key
  {
    double first;
    double second;
  };

  struct Entry
  {
    Key key;
    std::size_t cell;
  };

  struct EntryCompare
  {
    bool operator()(const Entry & a, const Entry & b) const;
  };

  void reset(
    const GridSnapshot & grid, std::size_t start, std::size_t goal,
    const SearchOptions & options);
  void updateVertex(std::size_t cell);
  void push(std::size_t cell);
  Key calculateKey(std::size_t cell) const;
  bool keyLess(const Key & a, const Key & b) const;
  bool computeShortestPath(SearchResult & result);
  SearchResult extractPath();
  bool optionsEqual(const SearchOptions & options) const;
  void applyChanges(const GridSnapshot & grid);
  bool cancelled() const;

  GridSnapshot grid_;
  SearchOptions options_;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::priority_queue<Entry, std::vector<Entry>, EntryCompare> open_;
  std::size_t start_{0};
  std::size_t last_start_{0};
  std::size_t goal_{0};
  double km_{0.0};
  bool initialized_{false};
  double changed_ratio_fallback_{0.35};
  FallbackReason last_fallback_reason_{FallbackReason::kNone};
  std::string last_fallback_detail_;
};

const char * fallbackReasonName(FallbackReason reason);

}  // namespace pongbot_local_graph_insertion_planner

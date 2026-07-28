#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace planner = pongbot_local_graph_insertion_planner;
namespace
{

struct Samples
{
  std::vector<double> milliseconds;
  std::vector<std::size_t> expanded;
};

double percentile(std::vector<double> values, double quantile)
{
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::ceil(quantile * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1)];
}

void printSummary(const std::string & mode, const Samples & samples)
{
  const double expansion_sum = std::accumulate(
    samples.expanded.begin(), samples.expanded.end(), 0.0);
  std::cout << std::fixed << std::setprecision(3)
            << "mode=" << mode
            << " samples=" << samples.milliseconds.size()
            << " median_ms=" << percentile(samples.milliseconds, 0.50)
            << " p95_ms=" << percentile(samples.milliseconds, 0.95)
            << " max_ms=" << *std::max_element(
    samples.milliseconds.begin(), samples.milliseconds.end())
            << " mean_expanded=" << expansion_sum / samples.expanded.size()
            << '\n';
}

template<typename Callback>
planner::SearchResult measure(Callback && callback, Samples & samples)
{
  const auto started = std::chrono::steady_clock::now();
  auto result = callback();
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  samples.milliseconds.push_back(elapsed);
  samples.expanded.push_back(result.expanded);
  return result;
}

void setObstacle(planner::GridSnapshot & grid, bool blocked)
{
  for (std::size_t y = 224; y <= 230; ++y) {
    for (std::size_t x = 228; x <= 234; ++x) {
      grid.costs[grid.index(x, y)] = blocked ? 253 : 0;
    }
  }
}

}  // namespace

int main()
{
  planner::GridSnapshot grid;
  grid.frame_id = "map";
  grid.size_x = 464;
  grid.size_y = 454;
  grid.resolution = 0.05;
  grid.costs.assign(grid.size_x * grid.size_y, 0);
  const auto start = grid.index(10, 227);
  const auto goal = grid.index(453, 227);
  planner::SearchOptions options;
  options.cost_penalty = 1.0;

  planner::DStarLite dstar(0.20);
  const auto initial = dstar.replan(grid, start, goal, options);
  if (initial.status != planner::SearchStatus::kSuccess) {
    std::cerr << "initial D* calculation failed\n";
    return 1;
  }

  Samples fresh_samples;
  Samples repair_samples;
  constexpr int kSamples = 30;
  for (int sample = 0; sample < kSamples; ++sample) {
    setObstacle(grid, sample % 2 == 0);

    const auto fresh = measure(
      [&]() {return planner::freshAstar(grid, start, goal, options);},
      fresh_samples);
    const auto repaired = measure(
      [&]() {return dstar.replan(grid, start, goal, options);},
      repair_samples);

    if (fresh.status != repaired.status ||
      fresh.status != planner::SearchStatus::kSuccess ||
      !planner::validPath(grid, repaired.path, options) ||
      std::abs(fresh.cost - repaired.cost) > 1e-9)
    {
      std::cerr << "differential mismatch at sample " << sample << '\n';
      return 2;
    }
  }

  std::cout << "grid_cells=" << grid.costs.size()
            << " changed_cells_per_event=49"
            << " differential_mismatches=0\n";
  printSummary("FRESH_ASTAR", fresh_samples);
  printSummary("DSTAR_REPAIR", repair_samples);
  return 0;
}

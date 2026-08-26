#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/graph/graph_types.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

/** @brief Deterministic mutable index for planar graph nodes. */
class SpatialIndex2D
{
public:
  virtual ~SpatialIndex2D() = default;
  virtual void clear() = 0;
  virtual void insert(NodeId id, Point2D position) = 0;
  virtual std::optional<NodeId> nearest(Point2D query) const = 0;
  virtual std::vector<NodeId> radiusSearch(Point2D query, double radius_m) const = 0;
};

/**
 * @brief Uniform-grid spatial hash with exact, deterministically ordered queries.
 *
 * Query results are ordered by squared distance and then NodeId. Re-inserting an
 * existing id replaces its position, so each NodeId has exactly one owner cell.
 */
class UniformGridSpatialIndex2D final : public SpatialIndex2D
{
public:
  explicit UniformGridSpatialIndex2D(double cell_size_m);
  void clear() override;
  void insert(NodeId id, Point2D position) override;
  std::optional<NodeId> nearest(Point2D query) const override;
  std::vector<NodeId> radiusSearch(Point2D query, double radius_m) const override;

private:
  struct BucketKey {int x; int y; bool operator==(const BucketKey & other) const noexcept;};
  struct BucketHash {std::size_t operator()(const BucketKey & key) const noexcept;};
  BucketKey bucket(Point2D point) const noexcept;

  double cell_size_m_;
  std::unordered_map<NodeId, Point2D> positions_;
  std::unordered_map<BucketKey, std::vector<NodeId>, BucketHash> buckets_;
};

}  // namespace rubi_heightmap_step_wavefront_planner

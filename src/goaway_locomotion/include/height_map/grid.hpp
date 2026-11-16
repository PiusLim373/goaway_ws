#pragma once

// The training's terrain scan (mjlab GridPatternCfg(size=(1.6, 1.0), resolution=0.1),
// ray_alignment="yaw"): meshgrid(x, y, indexing="xy") flattened, so x varies fastest and
// k = iy * kNx + ix. Fixed here, not a parameter, so it can't drift from the policy.

namespace height_map
{

constexpr int kNx = 17;                // x = -0.8 .. +0.8, back to front
constexpr int kNy = 11;                // y = -0.5 .. +0.5, right to left
constexpr int kNumPoints = kNx * kNy;  // 187
constexpr double kResolution = 0.1;
constexpr double kMinX = -0.8;
constexpr double kMinY = -0.5;

struct GridOffset
{
  double x;  // forward, in base_link turned with its heading only
  double y;  // left
};

constexpr GridOffset gridOffset(int k)
{
  return {kMinX + kResolution * (k % kNx), kMinY + kResolution * (k / kNx)};
}

}  // namespace height_map

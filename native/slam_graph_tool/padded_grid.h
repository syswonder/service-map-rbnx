#pragma once
#include <karto_sdk/Mapper.h>

// Keep endpoint cells: Karto rounds the maximum coordinate to index width,
// but its stock factory allocates only [0,width). Pad all sides one cell.
class PaddedGrid : public karto::OccupancyGrid {
 public:
  using karto::OccupancyGrid::OccupancyGrid;
  static karto::OccupancyGrid* Render(const karto::LocalizedRangeScanVector& scans,
      double resolution, kt_int32u minimum, double threshold) {
    if (scans.empty()) return nullptr;
    kt_int32s width,height; karto::Vector2<kt_double> offset;
    ComputeDimensions(scans,resolution,width,height,offset);
    offset.SetX(offset.GetX()-resolution); offset.SetY(offset.GetY()-resolution);
    auto* grid=new PaddedGrid(width+3,height+3,offset,resolution);
    grid->SetMinPassThrough(minimum);grid->SetOccupancyThreshold(threshold);
    grid->CreateFromScans(scans);
    return grid;
  }
};

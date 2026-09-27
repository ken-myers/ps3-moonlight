#pragma once

#include "src/platform/common.h"

namespace magewell {
  // Capture images contain contiguous NV12: Y at data, UV at data+row_pitch*height.
  // Width/height must match the requested encoder output; no scaling or RGB path.
  std::unique_ptr<platf::avcodec_encode_device_t> make_encode_device(int width, int height);
}

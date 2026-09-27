#include "magewell_encode.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_vaapi.h>
#include <va/va.h>
}

namespace magewell {
  namespace {
    int64_t monotonic_ns() {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    class nv12_encode_device_t final: public platf::avcodec_encode_device_t {
    public:
      nv12_encode_device_t(int width, int height): width_(width), height_(height) {
        // Sunshine uses this callback both to create VAAPI and to identify a
        // custom converter. A null pointer selects its RGB software converter.
        data = reinterpret_cast<void *>(&init_hardware);
        if (const char *path = std::getenv("MAGEWELL_TRACE")) {
          trace_fd_ = open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
        }
      }

      ~nv12_encode_device_t() override {
        av_frame_free(&source_);
        av_frame_free(&frame);
        if (trace_fd_ >= 0) {
          close(trace_fd_);
        }
      }

      static int init_hardware(platf::avcodec_encode_device_t *base, AVBufferRef **buffer) {
        auto &self = static_cast<nv12_encode_device_t &>(*base);
        if (self.colorspace.bit_depth != 8 || self.colorspace.full_range ||
            self.colorspace.colorspace != video::colorspace_e::rec709) {
          BOOST_LOG(error) << "Magewell NV12 is captured as limited-range BT.709; encoder metadata must match";
          return AVERROR(EINVAL);
        }
        const char *device = config::video.adapter_name.empty() ?
                               "/dev/dri/renderD128" : config::video.adapter_name.c_str();
        int result = av_hwdevice_ctx_create(buffer, AV_HWDEVICE_TYPE_VAAPI, device, nullptr, 0);
        if (result < 0) {
          BOOST_LOG(error) << "Magewell: cannot create VAAPI device " << device << ": " << result;
          return result;
        }
        auto *context = reinterpret_cast<AVHWDeviceContext *>((*buffer)->data);
        self.display_ = static_cast<AVVAAPIDeviceContext *>(context->hwctx)->display;
        BOOST_LOG(info) << "Magewell direct NV12 VAAPI: " << vaQueryVendorString(self.display_);
        return 0;
      }

      int set_frame(AVFrame *owned_frame, AVBufferRef *hw_frames_ctx) override {
        // Ownership transfers even when validation fails.
        av_frame_free(&frame);
        frame = owned_frame;
        if (!frame || !hw_frames_ctx || frame->format != AV_PIX_FMT_VAAPI ||
            frame->width != width_ || frame->height != height_) {
          BOOST_LOG(error) << "Magewell direct NV12 requires matching capture/encoder dimensions ("
                           << width_ << 'x' << height_ << "); scaling is not supported";
          return -1;
        }
        auto *context = reinterpret_cast<AVHWFramesContext *>(hw_frames_ctx->data);
        if (context->sw_format != AV_PIX_FMT_NV12) {
          BOOST_LOG(error) << "Magewell direct encoder supports only 8-bit NV12";
          return -1;
        }
        int result = av_hwframe_get_buffer(hw_frames_ctx, frame, 0);
        if (result < 0) {
          BOOST_LOG(error) << "Magewell: cannot allocate VAAPI NV12 surface: " << result;
          return result;
        }
        av_frame_free(&source_);
        source_ = av_frame_alloc();
        if (!source_) {
          return -1;
        }
        source_->format = AV_PIX_FMT_NV12;
        source_->width = width_;
        source_->height = height_;
        source_->color_range = frame->color_range;
        source_->colorspace = frame->colorspace;
        source_->color_primaries = frame->color_primaries;
        source_->color_trc = frame->color_trc;
        return 0;
      }

      int convert(platf::img_t &img) override {
        const auto start = monotonic_ns();
        if (!source_ || !frame || !img.data || img.width != width_ ||
            img.height != height_ || img.row_pitch < width_) {
          BOOST_LOG(error) << "Magewell: invalid NV12 image or unsupported dimension change";
          record(img, start, -1);
          return -1;
        }
        source_->data[0] = img.data;
        source_->data[1] = img.data + static_cast<size_t>(img.row_pitch) * img.height;
        source_->linesize[0] = img.row_pitch;
        source_->linesize[1] = img.row_pitch;
        // The CPU-backed capture image stays alive throughout this synchronous
        // upload. No intermediate RGB surface, swscale operation, or EGL context.
        int result = av_hwframe_transfer_data(frame, source_, 0);
        source_->data[0] = nullptr;
        source_->data[1] = nullptr;
        record(img, start, result);
        if (result < 0) {
          BOOST_LOG(error) << "Magewell: NV12 upload to VAAPI failed: " << result;
        }
        return result;
      }

      void init_codec_options(AVCodecContext *ctx, AVDictionary **options) override {
        const int client_slices = ctx->slices;
        int requested_slices = 1;
        if (const char *value = std::getenv("MAGEWELL_SLICES")) {
          char *end = nullptr;
          const long parsed = std::strtol(value, &end, 10);
          if (end != value && *end == '\0' && parsed >= 1 && parsed <= 16) {
            requested_slices = static_cast<int>(parsed);
          } else {
            BOOST_LOG(warning) << "Invalid MAGEWELL_SLICES; expected 1..16, using 1";
          }
        }
        ctx->slices = requested_slices;
        // Keep the existing Sunshine VAAPI policy: prefer LP when available,
        // query that entrypoint's actual RC capabilities, and respect max slices.
        VAProfile profile = VAProfileNone;
        if (ctx->codec_id == AV_CODEC_ID_H264) {
          profile = VAProfileH264High;
        } else if (ctx->codec_id == AV_CODEC_ID_HEVC && ctx->sw_pix_fmt == AV_PIX_FMT_NV12) {
          profile = VAProfileHEVCMain;
        } else if (ctx->codec_id == AV_CODEC_ID_AV1 && ctx->sw_pix_fmt == AV_PIX_FMT_NV12) {
          profile = VAProfileAV1Profile0;
        }
        if (!display_ || profile == VAProfileNone) {
          return;
        }
        int count = 0;
        std::vector<VAEntrypoint> supported(vaMaxNumEntrypoints(display_));
        if (vaQueryConfigEntrypoints(display_, profile, supported.data(), &count) != VA_STATUS_SUCCESS) {
          return;
        }
        supported.resize(count);
        VAEntrypoint entrypoint = static_cast<VAEntrypoint>(0);
        for (auto candidate : {VAEntrypointEncSliceLP, VAEntrypointEncSlice, VAEntrypointEncPicture}) {
          if (std::find(supported.begin(), supported.end(), candidate) != supported.end()) {
            entrypoint = candidate;
            break;
          }
        }
        if (!entrypoint) {
          return;
        }
        av_dict_set_int(options, "low_power", entrypoint == VAEntrypointEncSliceLP, 0);
        BOOST_LOG(info) << "Magewell VAAPI " << (entrypoint == VAEntrypointEncSliceLP ? "LP" : "normal")
                        << " encoding mode";

        VAConfigAttrib rc_attr = {VAConfigAttribRateControl};
        if (vaGetConfigAttributes(display_, profile, entrypoint, &rc_attr, 1) != VA_STATUS_SUCCESS ||
            rc_attr.value == VA_ATTRIB_NOT_SUPPORTED) {
          rc_attr.value = 0;
        }
        VAConfigAttrib slices = {VAConfigAttribEncMaxSlices};
        if (vaGetConfigAttributes(display_, profile, entrypoint, &slices, 1) != VA_STATUS_SUCCESS ||
            slices.value == VA_ATTRIB_NOT_SUPPORTED || slices.value == 0) {
          slices.value = 1;
        }
        if (ctx->slices > static_cast<int>(slices.value)) {
          ctx->slices = static_cast<int>(slices.value);
        }
        BOOST_LOG(info) << "Magewell VAAPI slices requested=" << requested_slices
                        << " effective=" << ctx->slices << " client-derived=" << client_slices;

        const char *vendor = vaQueryVendorString(display_);
        if (config::video.vaapi.strict_rc_buffer || (vendor && std::strstr(vendor, "Intel")) ||
            ctx->codec_id == AV_CODEC_ID_AV1) {
          ctx->rc_buffer_size = ctx->bit_rate * ctx->framerate.den / ctx->framerate.num;
          if (rc_attr.value & VA_RC_VBR) {
            av_dict_set(options, "rc_mode", "VBR", 0);
          } else if (rc_attr.value & VA_RC_CBR) {
            av_dict_set(options, "rc_mode", "CBR", 0);
          } else {
            av_dict_set(options, "rc_mode", "CQP", 0);
            av_dict_set_int(options, "qp", config::video.qp, 0);
            BOOST_LOG(info) << "Magewell VAAPI: using CQP; this entrypoint lacks VBR/CBR";
          }
        } else if (!(rc_attr.value & (VA_RC_CBR | VA_RC_VBR))) {
          av_dict_set(options, "rc_mode", "CQP", 0);
          av_dict_set_int(options, "qp", config::video.qp, 0);
        }
      }

    private:
      void record(const platf::img_t &img, int64_t start, int result) {
        const auto end = monotonic_ns();
        if (trace_fd_ < 0) {
          return;
        }
        const auto capture_ns = img.frame_timestamp ?
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    img.frame_timestamp->time_since_epoch()).count() : 0;
        char line[256];
        const int size = std::snprintf(line, sizeof(line), "encode_convert,%lld,%lld,%lld,%d,%d,%d\n",
          static_cast<long long>(start), static_cast<long long>(end), static_cast<long long>(capture_ns),
          img.width, img.height, result);
        if (size > 0 && size < static_cast<int>(sizeof(line))) {
          (void) write(trace_fd_, line, size);
        }
      }

      int width_, height_;
      VADisplay display_ = nullptr;  // Owned by Sunshine's FFmpeg device context.
      AVFrame *source_ = nullptr;  // Borrowed capture plane pointers only during convert().
      int trace_fd_ = -1;
    };
  }

  std::unique_ptr<platf::avcodec_encode_device_t> make_encode_device(int width, int height) {
    if (width <= 0 || height <= 0 || width % 2 || height % 2) {
      BOOST_LOG(error) << "Magewell NV12 dimensions must be positive and even";
      return nullptr;
    }
    return std::make_unique<nv12_encode_device_t>(width, height);
  }
}

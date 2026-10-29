#pragma once

#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#ifndef BOXI_HAS_TURBOJPEG
#define BOXI_HAS_TURBOJPEG 0
#endif

#if BOXI_HAS_TURBOJPEG
#include <turbojpeg.h>
#endif

namespace rectify_from_bag {
namespace turbojpeg_utils {

#if BOXI_HAS_TURBOJPEG
class TurboJpegHandle
{
public:
    explicit TurboJpegHandle(tjhandle handle) : handle_(handle) {}

    ~TurboJpegHandle()
    {
        if (handle_ != nullptr) {
            tjDestroy(handle_);
        }
    }

    TurboJpegHandle(const TurboJpegHandle&) = delete;
    TurboJpegHandle& operator=(const TurboJpegHandle&) = delete;

    tjhandle get() const { return handle_; }
    explicit operator bool() const { return handle_ != nullptr; }

private:
    tjhandle handle_ = nullptr;
};

struct TurboJpegBufferDeleter
{
    void operator()(unsigned char* buffer) const
    {
        if (buffer != nullptr) {
            tjFree(buffer);
        }
    }
};

using TurboJpegBuffer = std::unique_ptr<unsigned char, TurboJpegBufferDeleter>;
#endif

inline std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

inline bool formatLooksLikeJpeg(const std::string& format)
{
    const std::string lower = toLower(format);
    return lower.find("jpeg") != std::string::npos ||
           lower.find("jpg") != std::string::npos;
}

inline bool dataLooksLikeJpeg(const std::vector<uint8_t>& data)
{
    return data.size() >= 2 && data[0] == 0xFF && data[1] == 0xD8;
}

inline bool decodeCompressedBGROpenCV(const sensor_msgs::CompressedImage& msg, cv::Mat& bgr_out)
{
    if (msg.data.empty()) {
        return false;
    }

    cv::Mat buf(1, static_cast<int>(msg.data.size()), CV_8UC1,
                const_cast<uint8_t*>(msg.data.data()));
    bgr_out = cv::imdecode(buf, cv::IMREAD_COLOR);
    return !bgr_out.empty();
}

#if BOXI_HAS_TURBOJPEG
inline bool decodeCompressedBGRTurboJPEG(const sensor_msgs::CompressedImage& msg, cv::Mat& bgr_out)
{
    if (msg.data.empty()) {
        return false;
    }

    TurboJpegHandle handle(tjInitDecompress());
    if (!handle) {
        return false;
    }

    int width = 0;
    int height = 0;
    int subsamp = 0;
    int colorspace = 0;
    if (tjDecompressHeader3(handle.get(),
                            msg.data.data(),
                            static_cast<unsigned long>(msg.data.size()),
                            &width,
                            &height,
                            &subsamp,
                            &colorspace) != 0) {
        return false;
    }

    if (width <= 0 || height <= 0) {
        return false;
    }

    bgr_out.create(height, width, CV_8UC3);
    if (tjDecompress2(handle.get(),
                      msg.data.data(),
                      static_cast<unsigned long>(msg.data.size()),
                      bgr_out.data,
                      width,
                      static_cast<int>(bgr_out.step),
                      height,
                      TJPF_BGR,
                      0) != 0) {
        bgr_out.release();
        return false;
    }

    return true;
}
#endif

inline bool decodeCompressedBGR(const sensor_msgs::CompressedImage& msg, cv::Mat& bgr_out)
{
    if (msg.data.empty()) {
        return false;
    }

    const bool looks_like_jpeg =
        formatLooksLikeJpeg(msg.format) || dataLooksLikeJpeg(msg.data);

#if BOXI_HAS_TURBOJPEG
    if (looks_like_jpeg && decodeCompressedBGRTurboJPEG(msg, bgr_out)) {
        return true;
    }
#endif

    return decodeCompressedBGROpenCV(msg, bgr_out);
}

inline bool encodeJpegOpenCV(const cv::Mat& image, std::vector<uint8_t>& out, int quality)
{
    out.clear();
    if (image.empty()) {
        return false;
    }

    quality = std::max(1, std::min(100, quality));

    cv::Mat image_for_encode;
    if (image.type() == CV_8UC3 || image.type() == CV_8UC1) {
        image_for_encode = image;
    } else if (image.type() == CV_8UC4) {
        cv::cvtColor(image, image_for_encode, cv::COLOR_BGRA2BGR);
    } else {
        return false;
    }

    std::vector<int> params = {
        cv::IMWRITE_JPEG_QUALITY, quality
    };
    return cv::imencode(".jpg", image_for_encode, out, params);
}

#if BOXI_HAS_TURBOJPEG
inline bool encodeJpegTurboJPEG(const cv::Mat& image, std::vector<uint8_t>& out, int quality)
{
    out.clear();
    if (image.empty()) {
        return false;
    }

    int pixel_format = TJPF_UNKNOWN;
    int subsamp = TJSAMP_420;
    if (image.type() == CV_8UC3) {
        pixel_format = TJPF_BGR;
    } else if (image.type() == CV_8UC4) {
        pixel_format = TJPF_BGRA;
    } else if (image.type() == CV_8UC1) {
        pixel_format = TJPF_GRAY;
        subsamp = TJSAMP_GRAY;
    } else {
        return false;
    }

    quality = std::max(1, std::min(100, quality));

    TurboJpegHandle handle(tjInitCompress());
    if (!handle) {
        return false;
    }

    unsigned char* jpeg_buf_raw = nullptr;
    unsigned long jpeg_size = 0;
    const int rc = tjCompress2(handle.get(),
                               image.data,
                               image.cols,
                               static_cast<int>(image.step),
                               image.rows,
                               pixel_format,
                               &jpeg_buf_raw,
                               &jpeg_size,
                               subsamp,
                               quality,
                               0);
    TurboJpegBuffer jpeg_buf(jpeg_buf_raw);
    if (rc != 0) {
        return false;
    }

    out.assign(jpeg_buf.get(), jpeg_buf.get() + jpeg_size);
    return true;
}
#endif

inline bool encodeJpeg(const cv::Mat& image, std::vector<uint8_t>& out, int quality)
{
#if BOXI_HAS_TURBOJPEG
    if (encodeJpegTurboJPEG(image, out, quality)) {
        return true;
    }
#endif

    return encodeJpegOpenCV(image, out, quality);
}

}  // namespace turbojpeg_utils
}  // namespace rectify_from_bag

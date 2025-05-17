#include "depth_rvl.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <opencv2/core.hpp>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace nb = nanobind;
using nb::ndarray;

ndarray<nb::numpy, float>
decode(const ndarray<nb::numpy, nb::c_contig, uint8_t>& bytes,
       const std::string& format_field,
       const std::string& encoding = "32FC1")
{
    const size_t nbytes = bytes.size();
    // std::fprintf(stderr, "[nb.decode] bytes=%zu  fmt=\"%s\"\n",
    //              nbytes, format_field.c_str());

    std::vector<uint8_t> payload(nbytes);
    std::memcpy(payload.data(), bytes.data(), nbytes);

    if (nbytes >= 20) {
        uint32_t w = 0, h = 0;
        std::memcpy(&w, payload.data() + 12, 4);
        std::memcpy(&h, payload.data() + 16, 4);
        // std::fprintf(stderr, "             RVL header: %ux%u\n", w, h);
    }

    cv::Mat depth;
    try {
        depth = decode_depth(encoding, format_field, payload);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[nb.decode] decode_depth() error: %s\n", e.what());
        throw;
    }

    if (depth.empty() || depth.type() != CV_32FC1)
        throw std::runtime_error("decode_depth() failed or returned wrong type");

    /* ---- force copy to remove OpenCV internal alignment ------------------ */
    depth = depth.clone();

    /* ---- wrap into capsule-managed ndarray ------------------------------- */
    auto* mat_ptr = new cv::Mat(std::move(depth));
    nb::capsule owner(mat_ptr, [](void* p) noexcept {
        delete static_cast<cv::Mat*>(p);
    });

    const size_t  shape[2]  = { static_cast<size_t>(mat_ptr->rows),
                                static_cast<size_t>(mat_ptr->cols) };
    const int64_t stride[2] = {
        static_cast<int64_t>(mat_ptr->cols * sizeof(float)),  // row stride
        static_cast<int64_t>(sizeof(float))                   // col stride
    };

    // std::fprintf(stderr, "             out shape  : %zux%zu\n", shape[0], shape[1]);

    return nb::ndarray<nb::numpy, float>{
        mat_ptr->data,
        { static_cast<size_t>(mat_ptr->rows), static_cast<size_t>(mat_ptr->cols) },
        owner
    };
    
}

NB_MODULE(_compressed_depth, m)
{
    m.doc() = "Depth decoder for RVL / PNG compressed depth images";
    m.def("decode", &decode,
          nb::arg("payload"),
          nb::arg("format_field"),
          nb::arg("encoding") = "32FC1");
}

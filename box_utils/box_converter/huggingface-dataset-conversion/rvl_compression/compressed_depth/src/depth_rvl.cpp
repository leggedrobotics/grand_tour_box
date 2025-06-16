 #include "depth_rvl.hpp"

 #include <opencv2/imgcodecs.hpp>
 #include <opencv2/core/core.hpp>
 #include <limits>
 #include <cstring>
 #include <stdexcept>
 #include <vector>
 #include <iostream>
 #include <sstream>
 
 // ---------- enable/disable very verbose logging here -----------------
 #define RVL_DEBUG 0          // 0 = silent, 1 = high-level, 2 = per-loop
 // ---------------------------------------------------------------------
 
 static int* g_buffer;
 static int* g_pBuffer;
 static int  g_word;
 static int  g_nibblesWritten;
 
 static inline void RVL_DBG1(const std::string& s) { if constexpr (RVL_DEBUG >= 1) std::cerr << s << '\n'; }
 static inline void RVL_DBG2(const std::string& s) { if constexpr (RVL_DEBUG >= 2) std::cerr << s << '\n'; }
 
 static inline int DecodeVLE() {
     unsigned int nibble;
     int value = 0, bits = 29;
     do {
         if (!g_nibblesWritten) {
             g_word = *g_pBuffer++;       // load word
             g_nibblesWritten = 8;
         }
         nibble  = g_word & 0xf0000000;
         value  |= (nibble << 1) >> bits;
         g_word <<= 4;
         g_nibblesWritten--;
         bits -= 3;
     } while (nibble & 0x80000000);
     return value;
 }
 
 static inline void DecompressRVL(const unsigned char* input,
                                  unsigned short*      output,
                                  int                  numPixels)
 {
     g_buffer = g_pBuffer = (int*)input;
     g_nibblesWritten = 0;
 
     unsigned short current = 0, previous = 0;
     int remaining = numPixels;
 
     RVL_DBG2("[RVL]   starting decode loop, pixels=" + std::to_string(numPixels));
 
     while (remaining) {
         int zeros = DecodeVLE();
         if (zeros > remaining)
             throw std::runtime_error("RVL decode: zero-run exceeds bounds");
 
         remaining -= zeros;
         RVL_DBG2("        zeros = " + std::to_string(zeros));
         for (; zeros; --zeros) *output++ = 0;
 
         int nonzeros = DecodeVLE();
         if (nonzeros > remaining)
             throw std::runtime_error("RVL decode: non-zero run exceeds bounds");
 
         remaining -= nonzeros;
         RVL_DBG2("        nonzeros = " + std::to_string(nonzeros));
 
         for (; nonzeros; --nonzeros) {
             int positive = DecodeVLE();
             int delta    = (positive >> 1) ^ -(positive & 1);
             current      = previous + delta;
             *output++    = current;
             previous     = current;
         }
     }
     RVL_DBG2("[RVL]   decode loop finished");
 }
 
 /* ---------- helpers --------------------------------------------------------- */
 namespace {
 
 int bitDepth(const std::string& enc) {
     if (enc.rfind("32", 0) == 0) return 32;
     if (enc.rfind("16", 0) == 0) return 16;
     if (enc.rfind("8" , 0) == 0) return 8;
     return -1;
 }
 
 std::string compressionFormatFrom(const std::string& field) {
     if (field.empty()) return "png";
     if (field.find("compressedDepth png") != std::string::npos) return "png";
     if (field.find("compressedDepth rvl") != std::string::npos) return "rvl";
     if (field.find("compressedDepth") != std::string::npos &&
         field.find("compressedDepth ") == std::string::npos) return "png";
     throw std::runtime_error("Unsupported compressedDepth format: " + field);
 }
 
 } // unnamed namespace
 
 /* ---------- main decoder ---------------------------------------------------- */
 cv::Mat decode_depth(const std::string& encoding,
                      const std::string& format_field,
                      const std::vector<uint8_t>& payload)
 {
     if (payload.empty())
         throw std::runtime_error("decode_depth(): empty payload");
 
     const std::string cfmt  = compressionFormatFrom(format_field);
     const int bdepth        = bitDepth(encoding);
 
     if (bdepth <= 0)
         throw std::runtime_error("decode_depth(): unknown encoding \"" + encoding + '"');
 
     RVL_DBG1("[decode]  fmt=\"" + cfmt + "\"  encoding=\"" + encoding + "\"  payload=" + std::to_string(payload.size()) + " B");
 
     cv::Mat invDepth16;
 
     /* ---------- PNG path --------------------------------------------------- */
     if (cfmt == "png") {
         try {
             invDepth16 = cv::imdecode(payload, cv::IMREAD_UNCHANGED);
         } catch (const cv::Exception& e) {
             throw std::runtime_error("imdecode failed: " + std::string(e.what()));
         }
     }
     /* ---------- RVL path --------------------------------------------------- */
     else if (cfmt == "rvl") {
         constexpr size_t header_offset = 12;              // ConfigHeader
         if (payload.size() < header_offset + 8)
             throw std::runtime_error("RVL payload too small");
 
         const unsigned char* buf = payload.data() + header_offset;
         uint32_t cols = 0, rows = 0;
         std::memcpy(&cols, buf + 0, 4);
         std::memcpy(&rows, buf + 4, 4);
 
         RVL_DBG1("[decode]  RVL header  cols=" + std::to_string(cols) +
                  " rows=" + std::to_string(rows));
 
         if (cols == 0 || rows == 0)
             throw std::runtime_error("RVL header has zero dimension");
 
         const uint64_t numPix   = static_cast<uint64_t>(cols) * rows;
         const size_t   rvl_size = payload.size() - header_offset;
 
        //  if (numPix > std::numeric_limits<int>::max() ||
        //      numPix * 2ULL > rvl_size * 10ULL)
        //  {
        //      std::ostringstream oss;
        //      oss << "RVL sanity check failed: " << numPix
        //          << " pixels vs " << rvl_size << " bytes";
        //      throw std::runtime_error(oss.str());
        //  }
 
         invDepth16 = cv::Mat(rows, cols, CV_16UC1);
         DecompressRVL(buf + 8, invDepth16.ptr<unsigned short>(), rows * cols);
     }
     else {
         throw std::runtime_error("decode_depth(): unknown compression \"" + cfmt + '"');
     }
 
     if (invDepth16.empty())
         throw std::runtime_error("decode_depth(): decompression produced empty image");
 
     RVL_DBG1("[decode]  decompressed OK → converting to float32");
 
     /* ---------- inverse-depth → metres ------------------------------------ */
     constexpr float depthZ0  = 100.0f;
     constexpr float depthMax = 15.0f;
     const float depthQuantA  = depthZ0 * (depthZ0 + 1.0f);
     const float depthQuantB  = 1.0f - depthQuantA / depthMax;
 
     cv::Mat depth32(invDepth16.rows, invDepth16.cols, CV_32FC1);
     auto itOut = depth32.begin<float>();
     auto itIn  = invDepth16.begin<unsigned short>();
     for (; itOut != depth32.end<float>(); ++itOut, ++itIn) {
         *itOut = (*itIn != 0)
                   ? depthQuantA / (static_cast<float>(*itIn) - depthQuantB)
                   : std::numeric_limits<float>::quiet_NaN();
     }
 
     RVL_DBG1("[decode]  finished (rows=" + std::to_string(depth32.rows) +
              ", cols=" + std::to_string(depth32.cols) + ")");
     return depth32;
 }
 
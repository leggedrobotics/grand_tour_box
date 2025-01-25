#pragma once

//! C++ Standard Libraries
#include <optional>
#include <string>
#include <vector>

namespace colorless_uniform_mapper {

class CircularRingBuffer {
 public:
  //! Constructor
  explicit CircularRingBuffer(size_t ringSize = 0u);

  //! Add value to the circular ring buffer
  std::optional<std::string> addValue(std::string newValue);

  //! Get the current number of stored elements in the buffer.
  [[nodiscard]] size_t getNumberStoredValues() const;

 private:
  //! Ring buffer size
  size_t bufferSize_{0u};

  //! Actual ring buffer
  std::vector<std::string> ring_;

  //! Current location to write to into the ring buffer
  size_t currentIndex_{0u};

  //! Number of currently stored values in the ring (number_stored_values_<=ring_size_)
  size_t numberStoredValues_{0u};
};

}  // namespace colorless_uniform_mapper
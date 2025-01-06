
#include "colorless_uniform_mapper/circularRingBuffer.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace colorless_uniform_mapper {

CircularRingBuffer::CircularRingBuffer(size_t ringSize) : bufferSize_(ringSize), ring_(ringSize, std::string()) {}

std::optional<std::string> CircularRingBuffer::addValue(const std::string newMap) {
  bool remove{false};
  std::string toBeRemovedValue;
  // Check if the buffer is full
  if (numberStoredValues_ == bufferSize_) {
    // Get item to remove from the ring
    remove = true;
    toBeRemovedValue = ring_[currentIndex_];
  }
  ring_[currentIndex_] = newMap;
  // Move index forwards
  currentIndex_ = (currentIndex_ + 1) % bufferSize_;
  ++numberStoredValues_;
  numberStoredValues_ = std::min(numberStoredValues_, bufferSize_);
  if (remove) {
    // If we are removing a value, return it
    return toBeRemovedValue;
  }

  return {};
}

size_t CircularRingBuffer::getNumberStoredValues() const {
  return numberStoredValues_;
}

}  // namespace colorless_uniform_mapper
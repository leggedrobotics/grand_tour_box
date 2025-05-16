import numpy as np
import struct
import pickle


class RvlCodec:
    def __init__(self):
        self.buffer = None
        self.buffer_idx = 0
        self.word = 0
        self.nibbles_read = 0

    def decode_vle(self):
        """Decode a variable-length encoded value."""
        value = 0
        bits = 29
        while True:
            if self.nibbles_read == 0:
                # Load a new word (4 bytes)
                self.word = int.from_bytes(self.buffer[self.buffer_idx : self.buffer_idx + 4], byteorder="little")
                self.buffer_idx += 4
                self.nibbles_read = 8

            # Extract the highest nibble
            nibble = (self.word >> 28) & 0xF

            # Add to value
            value |= ((nibble << 1) & 0xF0) >> (bits - 3)

            # Shift for next nibble
            self.word <<= 4
            self.nibbles_read -= 1
            bits -= 3

            # Continue if highest bit of nibble is set
            if not (nibble & 0x8):
                break

        return value

    def decompress_rvl(self, input_data, output, num_pixels):
        """
        Decompress RVL-encoded data.

        Args:
            input_data: Bytes-like object containing the compressed data
            output: Numpy array to store the decompressed values
            num_pixels: Number of pixels to decode
        """
        self.buffer = input_data
        self.buffer_idx = 0
        self.nibbles_read = 0

        previous = 0
        pixels_decoded = 0

        while pixels_decoded < num_pixels:
            # Decode zeros
            zeros = self.decode_vle()
            for i in range(zeros):
                output[pixels_decoded] = 0
                pixels_decoded += 1

            # Decode non-zeros
            non_zeros = self.decode_vle()
            for i in range(non_zeros):
                positive = self.decode_vle()
                delta = (positive >> 1) ^ (-(positive & 1))
                current = previous + delta
                output[pixels_decoded] = current
                pixels_decoded += 1
                previous = current


def rvl_decode(data):
    """
    Decode RVL-encoded depth image data.

    Args:
        data: A bytes-like object containing the RVL-encoded image.

    Returns:
        A numpy array (uint16) containing the decoded depth image.
    """
    # Extract the dimensions from the header (first 8 bytes)
    cols = struct.unpack("<I", data[0:4])[0]  # Width
    rows = struct.unpack("<I", data[4:8])[0]  # Height

    # Sanity checks
    if rows == 0 or cols == 0:
        raise ValueError(f"Received malformed RVL-encoded image. Size {cols}x{rows} contains zero.")

    num_pixels = rows * cols
    # Check for potentially corrupted data
    if num_pixels > len(data) * 5:
        raise ValueError(f"Received malformed RVL-encoded image. It reports size {cols}x{rows}.")

    # Create output array
    output = np.zeros(num_pixels, dtype=np.uint16)

    # Decode the RVL data
    decoder = RvlCodec()
    decoder.decompress_rvl(data[8:], output, num_pixels)

    # Reshape to 2D image
    return output.reshape((rows, cols))


with open("/zedi2.pkl", "rb") as f:
    data = pickle.load(f)

out = rvl_decode(data)
print(out)

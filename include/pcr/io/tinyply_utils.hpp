#ifndef TINYPLY_UTILS_HPP
#define TINYPLY_UTILS_HPP

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>

/*
 This file is for the purpose of providing utility functions for my Ply wrapper
 to use

This header file contains useful helper functions/utilities for working with
tinyply.

These helper functions were taken from the tinyply github repository
https://github.com/ddiakopoulos/tinyply.
                                                                                  */
namespace pcr::io::ply::tinyply_utils {
struct float3 {
  float x, y, z;
};

inline std::vector<uint8_t> read_file_binary(const std::string &pathToFile) {
  std::ifstream file(pathToFile, std::ios::binary);
  std::vector<uint8_t> fileBufferBytes;

  if (file.is_open()) {
    file.seekg(0, std::ios::end);
    size_t sizeBytes = file.tellg();
    file.seekg(0, std::ios::beg);
    fileBufferBytes.resize(sizeBytes);
    if (file.read((char *)fileBufferBytes.data(), sizeBytes))
      return fileBufferBytes;
  } else
    throw std::runtime_error("could not open binary ifstream to path " +
                             pathToFile);
  return fileBufferBytes;
}

struct memory_buffer : public std::streambuf {
  char *p_start{nullptr};
  char *p_end{nullptr};
  size_t size;

  memory_buffer(char const *first_elem, size_t size)
      : p_start(const_cast<char *>(first_elem)), p_end(p_start + size),
        size(size) {
    setg(p_start, p_start, p_end);
  }

  pos_type seekoff(off_type off, std::ios_base::seekdir dir,
                   std::ios_base::openmode which) override {
    if (dir == std::ios_base::cur) {
      char *new_pos = gptr() + off;
      if (new_pos < p_start)
        new_pos = p_start;
      if (new_pos > p_end)
        new_pos = p_end;
      setg(p_start, new_pos, p_end);
    } else {
      char *new_pos = (dir == std::ios_base::beg ? p_start : p_end) + off;
      if (new_pos < p_start)
        new_pos = p_start;
      if (new_pos > p_end)
        new_pos = p_end;
      setg(p_start, new_pos, p_end);
    }
    return gptr() - p_start;
  }

  pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
    return seekoff(pos, std::ios_base::beg, which);
  }
};

struct memory_stream : virtual memory_buffer, public std::istream {
  memory_stream(char const *first_elem, size_t size)
      : memory_buffer(first_elem, size),
        std::istream(static_cast<std::streambuf *>(this)) {}
};
}; // namespace pcr::io::ply::tinyply_utils

#endif

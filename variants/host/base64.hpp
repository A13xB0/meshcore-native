// base64, with the API of densaugeo/base64 — one of MeshCore's lib_deps.
//
// Provided by the variant rather than fetched because it is forty lines of a
// format that has not changed since 1987, and a build that needs the network to
// resolve it fails offline for no reason. The API is matched exactly, including
// the detail that encode writes a NUL and does not count it.
#pragma once

#include <stdint.h>

namespace {

constexpr char b64_alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline unsigned char b64_index(unsigned char c) {
  if (c >= 'A' && c <= 'Z') return (unsigned char)(c - 'A');
  if (c >= 'a' && c <= 'z') return (unsigned char)(c - 'a' + 26);
  if (c >= '0' && c <= '9') return (unsigned char)(c - '0' + 52);
  if (c == '+') return 62;
  if (c == '/') return 63;
  return 0;  // includes '=' — padding contributes no bits
}

}  // namespace

// Room the encoder needs, excluding the trailing NUL it also writes.
inline unsigned int encode_base64_length(unsigned int input_length) {
  return (input_length + 2) / 3 * 4;
}

// An upper bound: the exact length depends on padding, which the caller learns
// from decode_base64's return value.
inline unsigned int decode_base64_length(unsigned int input_length) {
  return input_length / 4 * 3;
}

inline unsigned int encode_base64(const unsigned char* input, unsigned int input_length,
                                  unsigned char* output) {
  unsigned int o = 0;
  for (unsigned int i = 0; i < input_length; i += 3) {
    uint32_t v = (uint32_t)input[i] << 16;
    if (i + 1 < input_length) v |= (uint32_t)input[i + 1] << 8;
    if (i + 2 < input_length) v |= input[i + 2];

    output[o++] = (unsigned char)b64_alphabet[(v >> 18) & 0x3F];
    output[o++] = (unsigned char)b64_alphabet[(v >> 12) & 0x3F];
    output[o++] = i + 1 < input_length ? (unsigned char)b64_alphabet[(v >> 6) & 0x3F] : '=';
    output[o++] = i + 2 < input_length ? (unsigned char)b64_alphabet[v & 0x3F] : '=';
  }
  output[o] = '\0';
  return o;
}

inline unsigned int decode_base64(const unsigned char* input, unsigned int input_length,
                                  unsigned char* output) {
  unsigned int o = 0;
  for (unsigned int i = 0; i + 3 < input_length; i += 4) {
    uint32_t v = (uint32_t)b64_index(input[i]) << 18 | (uint32_t)b64_index(input[i + 1]) << 12 |
                 (uint32_t)b64_index(input[i + 2]) << 6 | b64_index(input[i + 3]);
    output[o++] = (unsigned char)(v >> 16);
    if (input[i + 2] != '=') output[o++] = (unsigned char)(v >> 8);
    if (input[i + 3] != '=') output[o++] = (unsigned char)v;
  }
  return o;
}

// The library's NUL-terminated overload, which is what MeshCore calls.
inline unsigned int decode_base64(const unsigned char* input, unsigned char* output) {
  unsigned int n = 0;
  while (input[n]) n++;
  return decode_base64(input, n, output);
}

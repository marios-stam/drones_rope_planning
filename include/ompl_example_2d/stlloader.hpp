// Single file header-only library for loading STL 3D models in either
// ASCII or binary formats.
//
// MIT License
//
// Copyright (c) 2019 David Cunningham
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef STLLOADER_H
#define STLLOADER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace stlloader
{

    ////////////////////////////////////////////////////////////////////////
    // External Interface
    ////////////////////////////////////////////////////////////////////////

    struct Vertex
    {
        float x, y, z;
    };
    bool vertices_equal(const stlloader::Vertex &a, const stlloader::Vertex &b);

    bool vertices_smaller(const stlloader::Vertex &a, const stlloader::Vertex &b);

    bool vertices_larger(const stlloader::Vertex &a, const stlloader::Vertex &b);
    struct Normal
    {
        float x, y, z;
    };

    struct Facet
    {
        Vertex vertices[3];
        Normal normal;
    };
    void print(const Facet &facet);

    struct Mesh
    {
        std::vector<Facet> facets;
        std::string name;
        std::string header;
    };
    void print(const Mesh &mesh);

    void parse_stream(std::istream &is, Mesh &mesh);
    void parse_stream(const char *filename, Mesh &mesh);

    enum class Format
    {
        ascii,
        binary
    };

    void write_stream(std::ostream &os, const Mesh &mesh, Format format = Format::ascii);
    void write_file(const char *filename, const Mesh &mesh, Format format = Format::ascii);

    // #ifdef STLLOADER_IMPLEMENTATION

    ////////////////////////////////////////////////////////////////////////
    // Printing
    ////////////////////////////////////////////////////////////////////////

    std::istream &operator>>(std::istream &is, Vertex &v);
    std::istream &operator>>(std::istream &is, Normal &n);

    void print(const Facet &facet);

    void print(const Mesh &mesh);

    ////////////////////////////////////////////////////////////////////////
    // Reading
    ////////////////////////////////////////////////////////////////////////

    void consume(std::istream &is, const std::string &expected);

    std::istringstream getlinestream(std::istream &is);

    void parse_ascii_facet(std::istream &is, Facet &facet);

    void parse_ascii_solid(std::istream &is, Mesh &mesh);

    void parse_ascii_stream(std::istream &is, Mesh &mesh);

    template <typename T> T little_endian_to_native(T v);

    template <> float little_endian_to_native(float v);

    template <typename T> T read_binary_value(std::istream &is);

    template <> uint16_t read_binary_value(std::istream &is);

    template <> uint32_t read_binary_value(std::istream &is);

    template <> float read_binary_value(std::istream &is);

    template <> Normal read_binary_value(std::istream &is);

    template <> Vertex read_binary_value(std::istream &is);

    const size_t STL_BINARY_HDR_SIZE = 80;

    const size_t STL_BINARY_META_SIZE = sizeof(uint32_t); // number of triangles

    const size_t STL_BINARY_TRIANGLE_SIZE = 3 * sizeof(float) +     // 1 normal
                                            3 * 3 * sizeof(float) + // 3 vertices
                                            sizeof(uint16_t);       // 1 attribute

    void parse_binary_stream(std::istream &is, Mesh &mesh);

    void parse_stream(std::istream &is, Mesh &mesh);

    void parse_file(const char *filename, Mesh &mesh);

    ////////////////////////////////////////////////////////////////////////
    // Writing
    ////////////////////////////////////////////////////////////////////////

    std::ostream &operator<<(std::ostream &os, const Vertex &v);
    std::ostream &operator<<(std::ostream &os, const Normal &n);

    template <typename T> T native_to_little_endian(T v);

    template <> float native_to_little_endian(float v);

    template <typename T> void write_binary_value(std::ostream &os, const T &value);

    template <> void write_binary_value(std::ostream &os, const uint16_t &value);

    template <> void write_binary_value(std::ostream &os, const uint32_t &value);

    template <> void write_binary_value(std::ostream &os, const float &value);

    template <> void write_binary_value(std::ostream &os, const Vertex &v);

    template <> void write_binary_value(std::ostream &os, const Normal &n);

    void write_stream(std::ostream &os, const Mesh &mesh, Format format);

    void write_file(const char *filename, const Mesh &mesh, Format format);

} // namespace stlloader

#endif // STLLOADER_H
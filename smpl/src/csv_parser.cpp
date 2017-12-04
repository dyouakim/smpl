////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/csv_parser.h>

// standard includes
#include <stdexcept>

#define CSV_DEBUG 0
#if CSV_DEBUG
#include <smpl/console/console.h>
#define PRINT(fmt, ...) SMPL_DEBUG(fmt, ##__VA_ARGS__)
#else
#define PRINT(fmt, ...)
#endif

namespace sbpl {

bool CSVParser::parseStream(std::istream& s, bool has_header)
{
    m_has_header = has_header;

    m_fields.clear();

    size_t fields_per_record = 0;

    std::istream::pos_type pos = s.tellg();

    if (has_header && !parseOptionalHeader(s, pos, m_fields)) {
        PRINT("Failed to parse optional header");
        return false;
    }

    if (has_header) {
        fields_per_record = m_fields.size();
    }

    if (!parseRecord(s, pos, m_fields)) {
        PRINT("Failed to parse head record");
        return false;
    }

    if (!has_header) {
        fields_per_record = m_fields.size();
    }

    bool done = false;
    while (!done) {
        std::istream::pos_type prev_pos = pos;
        size_t prev_num_fields = m_fields.size();
        if (!parseCRLF(s, pos)) {
            done = true;
        } else if (!parseRecord(s, pos, m_fields)) {
            pos = prev_pos;
            done = true;
        } else {
            if (m_fields.size() != prev_num_fields + fields_per_record) {
                PRINT("Record contains insufficient fields (expected: %zu, actual: %zu)", fields_per_record, m_fields.size() - prev_num_fields);
                m_fields.resize(prev_num_fields);
            }
            PRINT("Parsed tail record");
        }
    }

    // skip checking for optional crlf
    if (!parseCRLF(s, pos)) {

    }

    m_field_count = fields_per_record;
    return true;
}

const std::string& CSVParser::nameAt(size_t ni) const
{
    if (!m_has_header) {
        throw std::out_of_range("Invalid index for csv header name");
    }
    return m_fields[ni];
}

const std::string& CSVParser::fieldAt(size_t ri, size_t fi) const
{
    if ((ri + 1) * m_field_count + fi >= m_fields.size()) {
        throw std::out_of_range("Invalid indices for csv field");
    }
    return m_fields[(ri + 1) * m_field_count + fi];
}

bool CSVParser::parseRecord(
    std::istream& s,
    std::istream::pos_type& pos,
    std::vector<std::string>& header)
{
    PRINT("Parse record");

    std::string name;
    if (!parseField(s, pos, name)) {
        PRINT("Failed to parse record");
        return false;
    }
    header.push_back(name);

    bool done = false;
    while (!done) {
        std::istream::pos_type prev_pos = pos;
        if (!parseComma(s, pos)) {
            done = true;
        } else if (!parseField(s, pos, name)) {
            pos = prev_pos;
            done = true;
        } else {
            header.push_back(name);
        }
    }

    PRINT("Parsed record");
    return true;
}

bool CSVParser::parseField(
    std::istream& s,
    std::istream::pos_type& pos,
    std::string& name)
{
    PRINT("Parse field");
    name.clear();
    if (parseEscaped(s, pos, name)) {
        PRINT("Parsed escaped field '%s'", name.c_str());
        return true;
    } else if (parseNonEscaped(s, pos, name)) {
        PRINT("Parsed non-escaped field '%s'", name.c_str());
        return true;
    } else {
        PRINT("Failed to parse field");
        return false;
    }
}

bool CSVParser::parseEscaped(
    std::istream& s,
    std::istream::pos_type& pos,
    std::string& text)
{
    std::istream::pos_type prev_pos = pos;
    text.clear();
    if (!parseDoubleQuote(s, pos)) {
        return false;
    }
    std::istream::traits_type::char_type c;
    bool done = false;
    while (!done) {
        if (parseTextData(s, pos, c)) {
            text.push_back(c);
        } else if (parseComma(s, pos)) {
            text.push_back(',');
        } else if (parseCR(s, pos)) {
            text.push_back('\r');
        } else if (parseLF(s, pos)) {
            text.push_back('\n');
        } else if (parseDoubleQuote2(s, pos)) {
            text.push_back('"');
        } else {
            done = true;
        }
    }
    if (!parseDoubleQuote(s, pos)) {
        pos = prev_pos;
        return false;
    }
    return true;
}

bool CSVParser::parseNonEscaped(
    std::istream& s,
    std::istream::pos_type& pos,
    std::string& text)
{
    text.clear();
    std::istream::traits_type::char_type c;
    while (parseTextData(s, pos, c)) {
        text.push_back(c);
    }
    return true;
}

bool CSVParser::parseOptionalHeader(
    std::istream& s,
    std::istream::pos_type& pos,
    std::vector<std::string>& header)
{
    PRINT("Parse optional header");

    std::istream::pos_type prev_pos = pos;

    if (!parseHeader(s, pos, header)) {
        PRINT("Failed to parse optional header");
        return false;
    }

    if (!parseCRLF(s, pos)) {
        PRINT("Failed to parse optional header");
        pos = prev_pos;
        return false;
    }

    PRINT("Parsed optional header");
    return true;
}

bool CSVParser::parseHeader(
    std::istream& s,
    std::istream::pos_type& pos,
    std::vector<std::string>& header)
{
    PRINT("Parse header");

    std::string name;
    if (!parseName(s, pos, name)) {
        PRINT("Failed to parse head name");
        return false;
    }
    header.push_back(name);

    bool done = false;
    while (!done) {
        std::istream::pos_type prev_pos = pos;
        if (!parseComma(s, pos)) {
            done = true;
        } else if (!parseName(s, pos, name)) {
            pos = prev_pos;
            done = true;
        } else {
            header.push_back(name);
        }
    }

    PRINT("Parsed header");
    return true;
}

bool CSVParser::parseTextData(
    std::istream& s,
    std::istream::pos_type& pos,
    std::istream::char_type& co)
{
    s.seekg(pos);
    std::istream::traits_type::int_type c = s.peek();
    if ((c >= 0x20 && c <= 0x20) ||
        (c >= 0x23 && c <= 0x2B) ||
        (c >= 0x2D && c <= 0x7E))
    {
        co = c;
        s.seekg(1, std::ios_base::cur);
        pos = s.tellg();
        return true;
    } else {
        return false;
    }
}

bool CSVParser::parseDoubleQuote(
    std::istream& s,
    std::istream::pos_type& pos)
{
    PRINT("Parse double quote");
    s.seekg(pos);
    if (s.peek() == 0x22) {
        s.seekg(1, std::ios_base::cur);
        pos = s.tellg();
        PRINT("Parsed double quote");
        return true;
    } else {
        PRINT("Failed to parse double quote");
        return false;
    }
}

bool CSVParser::parseDoubleQuote2(
    std::istream& s,
    std::istream::pos_type& pos)
{
    PRINT("Parse double double quote");
    std::istream::pos_type prev_pos = pos;
    if (!parseDoubleQuote(s, pos)) {
        PRINT("Failed to parse double double quote");
        return false;
    }
    if (!parseDoubleQuote(s, pos)) {
        pos = prev_pos;
        PRINT("Failed to parse double double quote");
        return false;
    }
    PRINT("Parsed double double quote");
    return true;
}

bool CSVParser::parseComma(
    std::istream& s,
    std::istream::pos_type& pos)
{
    PRINT("Parse comma");
    s.seekg(pos);
    if (s.peek() == 0x2C) {
        s.seekg(1, std::ios_base::cur);
        pos = s.tellg();
        PRINT("Parsed comma");
        return true;
    } else {
        PRINT("Failed to parse comma");
        return false;
    }
}

bool CSVParser::parseCR(
    std::istream& s,
    std::istream::pos_type& pos)
{
    s.seekg(pos);
    if (s.peek() == 0x0D) {
        s.seekg(1, std::ios_base::cur);
        pos = s.tellg();
        return true;
    } else {
        return false;
    }
}

bool CSVParser::parseLF(
    std::istream& s,
    std::istream::pos_type& pos)
{
    s.seekg(pos);
    if (s.peek() == 0x0A) {
        s.seekg(1, std::ios_base::cur);
        pos = s.tellg();
        return true;
    } else {
        return false;
    }
}

bool CSVParser::parseCRLF(
    std::istream& s,
    std::istream::pos_type& pos)
{
    PRINT("Parse CRLF");
    std::istream::pos_type prev_pos = pos;
//    if (!parseCR(s, pos)) {
//        PRINT("Failed to parse CRLF");
//        return false;
//    }
    if (!parseLF(s, pos)) {
        pos = prev_pos;
        PRINT("Failed to parse CRLF");
        return false;
    }
    PRINT("Parsed CRLF");
    return true;
}

bool CSVParser::parseName(
    std::istream& s,
    std::istream::pos_type& pos,
    std::string& name)
{
    return parseField(s, pos, name);
}

} // namespace sbpl

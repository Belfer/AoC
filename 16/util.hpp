#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <stdexcept>

namespace util
{
    class stopwatch_t
    {
    public:
        using clock_t = std::chrono::steady_clock;
        using time_point_t = clock_t::time_point;
        using time_span_t = std::chrono::nanoseconds;

        inline void start() noexcept
        {
            m_start = clock_t::now();
        }

        template <typename DurationType = std::chrono::duration<float>>
        inline DurationType elapsed() const noexcept
        {
            return std::chrono::duration_cast<DurationType>(clock_t::now() - m_start);
        }

        inline int elapsed_ms() const noexcept
        {
            return elapsed<std::chrono::duration<int, std::milli>>().count();
        }

    private:
        time_point_t m_start{ clock_t::now() };
    };

	static std::vector<std::string> read_file(const char* filepath)
	{
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file.");
        }

        std::vector<std::string> lines;
        std::string line;

        while (std::getline(file, line))
        {
            if (!line.empty())
            {
                lines.push_back(line);
            }
        }

        if (lines.empty())
        {
            throw std::runtime_error("File is empty.");
        }
        return lines;
	}

    static void write_file(const char* filepath, const std::string& text)
    {
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file for writing.");
        }

        file << text;

        if (!file)
        {
            throw std::runtime_error("Failed to write to file.");
        }
    }
}
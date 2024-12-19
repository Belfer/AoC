#include <util.hpp>

#include <queue>
#include <unordered_map>
#include <cmath>

using u8 = unsigned char;
enum struct tile_e : u8 { invalid, empty, wall, start, end };
enum struct dir_e : u8 { n, s, e, w, none, path = 0xF0 };

struct ivec2
{
    int x{ 0 }, y{ 0 };
    inline ivec2 operator+(const ivec2& o) const { return ivec2{ x + o.x, y + o.y }; }
    inline ivec2 operator-(const ivec2& o) const { return ivec2{ x - o.x, y - o.y }; }
};

struct state_t
{
    static constexpr ivec2 moves[] =
    {
        ivec2{ 0, -1},   // North
        ivec2{ 0,  1},   // South
        ivec2{ 1,  0},   // East
        ivec2{-1,  0}    // West
    };

    dir_e dir{ dir_e::none };
    int g_cost{ 0 };
    int h_cost{ 0 };
    int p_idx{ 0 };

    inline int f_cost() const { return g_cost + h_cost; }
    inline bool operator>(const state_t& other) const
    {
        if (f_cost() == other.f_cost())
            return h_cost > other.h_cost;
        return f_cost() > other.f_cost();
    }

    inline void reset()
    {
        dir = dir_e::none;
        g_cost = 0;
        h_cost = 0;
        p_idx = 0;
    }
};

struct tile_t
{
    tile_e type{ tile_e::empty };
    ivec2 pos{ 0, 0 };
    state_t state{};
};

struct maze_t
{
    ivec2 size{ 0, 0 };
    tile_t* map{ nullptr };
    int solve_time{ 0 };
    int search_count{ 0 };
    int path_cost{ 0 };

    inline std::size_t idx(int x, int y) const { return (std::size_t)y * size.x + x; }
    inline tile_t& get(int x, int y) { return map[idx(x, y)]; }
    inline const tile_t& get(int x, int y) const { return map[idx(x, y)]; }

    static constexpr tile_e char_to_tile(char c);
    static constexpr char tile_to_char(const tile_t& t);
    
    static int heuristic(const tile_t& a, const tile_t& b);

    void load(const char* filepath);
    void unload();

    void print(const char* filepath) const;
    void solve();
};

constexpr tile_e maze_t::char_to_tile(char c)
{
    switch (c) {
    case '.': return tile_e::empty;
    case '#': return tile_e::wall;
    case 'S': return tile_e::start;
    case 'E': return tile_e::end;
    default: return tile_e::invalid;
    }
}

constexpr char maze_t::tile_to_char(const tile_t& t)
{
    const dir_e masked_dir = (dir_e)((int)t.state.dir ^ (int)dir_e::path);
    const bool is_path = ((int)t.state.dir & (int)dir_e::path) > 0;

    if (is_path && t.type != tile_e::start && t.type != tile_e::end)
    {
        switch (masked_dir)
        {
        case dir_e::n: return '^';
        case dir_e::s: return 'v';
        case dir_e::e: return '>';
        case dir_e::w: return '<';
        default: return '?';
        }
    }

    switch (t.type)
    {
    case tile_e::empty: return '.';
    case tile_e::wall: return '#';
    case tile_e::start: return 'S';
    case tile_e::end: return 'E';
    default: return '?';
    }
}

int maze_t::heuristic(const tile_t& a, const tile_t& b)
{
    // Compute Manhattan distance
    int manhattan_dist = std::abs(a.pos.x - b.pos.x) + std::abs(a.pos.y - b.pos.y);

    // Check if the path is straight
    bool is_straight_path = (a.pos.x == b.pos.x || a.pos.y == b.pos.y);

    // Determine desired direction to align with the goal
    dir_e desired_dir =
        (a.pos.x < b.pos.x) ? dir_e::e :
        (a.pos.x > b.pos.x) ? dir_e::w :
        (a.pos.y < b.pos.y) ? dir_e::s :
        dir_e::n;

    // Calculate rotation cost if the current direction doesn't match the desired one
    int rotation_cost = (a.state.dir != desired_dir) ? 1000 : 0;

    // Add a base rotation cost if the path is not straight
    if (!is_straight_path)
        rotation_cost += 1000;

    // Return the total heuristic
    return manhattan_dist + rotation_cost;
}

void maze_t::load(const char* filepath)
{
    const std::vector<std::string> lines = util::read_file(filepath);

    size.y = static_cast<int>(lines.size());
    size.x = static_cast<int>(lines[0].size());

    for (const auto& l : lines)
    {
        if (l.size() != static_cast<size_t>(size.x))
        {
            throw std::runtime_error("Inconsistent row lengths in maze file.");
        }
    }

    map = new tile_t[size.x * size.y];

    for (int y = 0; y < size.y; ++y)
    {
        for (int x = 0; x < size.x; ++x)
        {
            auto& t = get(x, y);
            t = tile_t{ char_to_tile(lines[y][x]), ivec2{x, y}, dir_e::none, 0, 0 };

            if (t.type == tile_e::invalid)
                throw std::invalid_argument("Invalid character in maze file.");
        }
    }
}

void maze_t::unload()
{
    delete[] map;
}

void maze_t::print(const char* filepath) const
{
    std::ostringstream oss;
    oss << "Dimensions: " << size.x << " x " << size.y << std::endl;
    oss << "Solved in: " << solve_time << " ms. Search count: " << search_count << std::endl;
    oss << "Best path cost " << path_cost << " points" << std::endl;

    // Short output for console
    std::cout << oss.str();

    // Full output for file
    for (int y = 0; y < size.y; ++y)
    {
        for (int x = 0; x < size.x; ++x)
        {
            const auto& t = get(x, y);
            char c = tile_to_char(t);
            oss << c;
        }
        oss << '\n';
    }

    util::write_file(filepath, oss.str());
}

void maze_t::solve()
{
    // Reset map state
    solve_time = 0;
    search_count = 1;
    path_cost = -1;

    util::stopwatch_t sw{};
    sw.start();

    // Locate the start and end tiles
    int start_idx = -1, end_idx = -1;
    for (int i = 0; i < size.x * size.y; ++i)
    {
        map[i].state.reset();
        if (map[i].type == tile_e::start) { start_idx = i; }
        if (map[i].type == tile_e::end) { end_idx = i; }
    }

    if (start_idx == -1 || end_idx == -1)
    {
        throw std::runtime_error("Maze must have a start (S) and an end (E).");
    }

    // Priority queue for A* search
    auto priority_fn = [&](int a, int b) { return map[a].state > map[b].state; };
    std::priority_queue<int, std::vector<int>, decltype(priority_fn)> pq(priority_fn);

    // Initialize A* with the starting tile
    tile_t& start_tile = map[start_idx];
    start_tile.state.dir = dir_e::e;

    pq.push(start_idx);
    while (!pq.empty())
    {
        int current_idx = pq.top();
        pq.pop();

        tile_t& current = map[current_idx];

        // If we reached the end, return the cost
        if (current_idx == end_idx)
        {
            path_cost = current.state.g_cost;
            break;
        }

        // Explore neighboring tiles
        for (int move_dir = 0; move_dir < 4; ++move_dir)
        {
            ivec2 n = current.pos + state_t::moves[move_dir];

            // Ensure the move is within bounds
            if (n.x < 0 || n.x >= size.x || n.y < 0 || n.y >= size.y)
                continue;

            int neighbor_idx = idx(n.x, n.y);
            tile_t& neighbor = map[neighbor_idx];

            // Skip walls
            if (neighbor.type == tile_e::wall)
                continue;

            // Calculate the cost of moving to this neighbor
            int move_cost = current.state.dir != static_cast<dir_e>(move_dir) ? 1001 : 1;
            int g_cost = current.state.g_cost + move_cost;

            // If we found a cheaper path to this neighbor, update it
            if (neighbor.state.g_cost == 0 || g_cost < neighbor.state.g_cost)
            {
                neighbor.state.dir = static_cast<dir_e>(move_dir); // Track direction
                neighbor.state.g_cost = g_cost;
                neighbor.state.h_cost = heuristic(neighbor, map[end_idx]);
                neighbor.state.p_idx = current_idx;
                pq.push(neighbor_idx);
                search_count++;
            }
        }
    }

    // Update solve time
    solve_time = sw.elapsed_ms();

    // Ensure we found a valid path
    if (path_cost == -1)
    {
        std::cerr << "No path found to the goal." << std::endl;
    }
    // Format map to only show valid path
    else
    {
        int curr_idx = end_idx;
        while (true)
        {
            map[curr_idx].state.dir = (dir_e)((int)map[curr_idx].state.dir | (int)dir_e::path);
            curr_idx = map[curr_idx].state.p_idx;

            if (curr_idx == start_idx)
                break;
        }
    }
}

int main(int argc, char** args)
{
    /*
    * Example 1: 7036
    * Example 2: 11048
    * Input: 107476
    */

    try
    {
        maze_t maze{};
        maze.load(WD"/input.txt");
        maze.solve();
        maze.print(WD"/output.txt");
        maze.unload();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

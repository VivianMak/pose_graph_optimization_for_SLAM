#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "utils.hpp"

// -------------------------
// viz: header-only trajectory dumping
// -------------------------
//
// Turns node lists into (a) a plain x,y,theta CSV that the python scripts in
// visualization/ can read and (b) a standalone SVG that opens in any browser.
// Header-only on purpose so it needs no new library or CMake target.
//
namespace viz {

// -------------------------
// Style: how a trajectory is drawn
// -------------------------
//
// Line   - connected polyline, for the sparse pose-graph routes (~42 nodes)
// Points - unconnected dots, for dense raw data like the full odometry stream
//          (~4000 samples), where connecting them would just look like a line
//          and hide the sample scatter.
//
enum class Style { Line, Points };

// -------------------------
// Trajectory: one named route to draw
// -------------------------
struct Trajectory {
    std::string name;               // label drawn in the SVG legend
    std::string color;              // any CSS color, e.g. "crimson"
    std::vector<utils::Pose> poses; // route in world meters
    Style style;                    // line or scatter

    Trajectory(std::string name_, std::string color_, std::vector<utils::Pose> poses_,
               Style style_ = Style::Line)
        : name(std::move(name_)), color(std::move(color_)), poses(std::move(poses_)),
          style(style_) {}
};

/**
 * Pull poses out of the owned node list produced by PoseGraph.
 *
 * @param nodes  (vector) nodes as unique_ptr, in build order
 * @return poses in the same order as the nodes
 */
inline std::vector<utils::Pose>
posesFromNodes(const std::vector<std::unique_ptr<utils::Node>>& nodes) {
    std::vector<utils::Pose> poses;
    poses.reserve(nodes.size());
    for (const auto& n : nodes) {
        poses.push_back(n->pose);
    }
    return poses;
}

/**
 * Pull poses out of a value node list, e.g. GnOptimizer::getX().
 *
 * X_ is stored reversed inside the optimizer, so sorting by node_id is what
 * puts the optimized route back in the same order as the pre-optimized one.
 * Without this the two routes are drawn end-to-start against each other.
 *
 * @param nodes      (vector) nodes by value
 * @param sort_by_id (bool) restore build order using node_id
 * @return poses ordered to match the original pose graph
 */
inline std::vector<utils::Pose>
posesFromNodes(const std::vector<utils::Node>& nodes, bool sort_by_id = true) {
    std::vector<const utils::Node*> ordered;
    ordered.reserve(nodes.size());
    for (const auto& n : nodes) {
        ordered.push_back(&n);
    }

    if (sort_by_id) {
        std::sort(ordered.begin(), ordered.end(),
                  [](const utils::Node* a, const utils::Node* b) {
                      return a->node_id < b->node_id;
                  });
    }

    std::vector<utils::Pose> poses;
    poses.reserve(ordered.size());
    for (const auto* n : ordered) {
        poses.push_back(n->pose);
    }
    return poses;
}

/**
 * Write poses to a CSV with an x,y,theta header.
 *
 * Matches the column layout of data/odom_data.csv so the existing
 * visualization/ scripts can load the output with no changes.
 *
 * @param path   (string) file to write
 * @param poses  (vector) route to dump
 * @return true if the file was written
 */
inline bool writeCSV(const std::string& path, const std::vector<utils::Pose>& poses) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "viz: could not open " << path << " for writing\n";
        return false;
    }

    out << "x,y,theta\n";
    for (const auto& p : poses) {
        out << p.x << "," << p.y << "," << p.theta << "\n";
    }

    std::cout << "viz: wrote " << poses.size() << " poses to " << path << "\n";
    return true;
}

/**
 * Read an x,y,theta CSV back into poses.
 *
 * Lets the SVG overlay data that never passes through the pose graph, such as
 * the raw noisy odometry stream in data/noisy_odom_data.csv. A leading header
 * row is detected and skipped; a missing theta column defaults to 0.
 *
 * @param path  (string) file to read
 * @return poses in file order, empty if the file could not be opened
 */
inline std::vector<utils::Pose> readCSV(const std::string& path) {
    std::vector<utils::Pose> poses;

    std::ifstream in(path);
    if (!in) {
        std::cerr << "viz: could not open " << path << " for reading\n";
        return poses;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cx, cy, ct;
        if (!std::getline(ss, cx, ',')) continue;
        if (!std::getline(ss, cy, ',')) continue;
        std::getline(ss, ct, ',');  // theta is optional

        utils::Pose p{};
        try {
            p.x = std::stod(cx);
            p.y = std::stod(cy);
            p.theta = ct.empty() ? 0.0 : std::stod(ct);
        } catch (const std::exception&) {
            continue;  // header row or malformed line
        }
        poses.push_back(p);
    }

    std::cout << "viz: read " << poses.size() << " poses from " << path << "\n";
    return poses;
}

// -------------------------
// Internal: world -> pixel mapping
// -------------------------
namespace detail {

struct Viewport {
    double scale;    // pixels per meter, same on both axes
    double min_x;    // world-space origin of the drawn region
    double min_y;
    double off_x;    // pixel offset that centers the route
    double off_y;
    int height;      // needed to flip y for screen coords

    double px(double x) const { return off_x + (x - min_x) * scale; }
    double py(double y) const { return height - off_y - (y - min_y) * scale; }
};

/**
 * Fit every trajectory into one shared, aspect-preserving viewport.
 *
 * Both routes must share a viewport or the drift between them is invisible.
 *
 * @param trajs   (vector) all routes being drawn
 * @param width   (int) canvas width in pixels
 * @param height  (int) canvas height in pixels
 * @param margin  (double) blank border in pixels
 * @return viewport mapping world meters to pixels
 */
inline Viewport fit(const std::vector<Trajectory>& trajs,
                    int width, int height, double margin) {
    double min_x = 1e300, max_x = -1e300;
    double min_y = 1e300, max_y = -1e300;

    for (const auto& t : trajs) {
        for (const auto& p : t.poses) {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);
            max_y = std::max(max_y, p.y);
        }
    }

    // Degenerate or empty input still has to produce a usable viewport
    if (min_x > max_x) {
        min_x = max_x = min_y = max_y = 0.0;
    }

    double span_x = std::max(max_x - min_x, 1e-9);
    double span_y = std::max(max_y - min_y, 1e-9);

    double usable_w = width - 2.0 * margin;
    double usable_h = height - 2.0 * margin;

    // One scale for both axes keeps the plot square, like plt.axis("equal")
    double scale = std::min(usable_w / span_x, usable_h / span_y);

    Viewport vp;
    vp.scale  = scale;
    vp.min_x  = min_x;
    vp.min_y  = min_y;
    vp.off_x  = margin + (usable_w - span_x * scale) / 2.0;
    vp.off_y  = margin + (usable_h - span_y * scale) / 2.0;
    vp.height = height;
    return vp;
}

} // namespace detail

/**
 * Draw every trajectory into a single standalone SVG file.
 *
 * Each route becomes a polyline, with a filled dot at the start and a hollow
 * ring at the end so the direction of travel is readable.
 *
 * @param path    (string) file to write
 * @param trajs   (vector) routes to overlay, drawn in order
 * @param width   (int) canvas width in pixels
 * @param height  (int) canvas height in pixels
 * @param margin  (double) blank border in pixels
 * @return true if the file was written
 */
inline bool writeSVG(const std::string& path,
                     const std::vector<Trajectory>& trajs,
                     int width = 900,
                     int height = 700,
                     double margin = 48.0) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "viz: could not open " << path << " for writing\n";
        return false;
    }

    const detail::Viewport vp = detail::fit(trajs, width, height, margin);

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
        << height << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";

    for (const auto& t : trajs) {
        if (t.poses.empty()) continue;

        if (t.style == Style::Points) {
            // A zero-length subpath ("h0") with a round linecap renders as a
            // dot. One path element for thousands of samples instead of one
            // <circle> each keeps the file a fraction of the size.
            out << "  <path fill=\"none\" stroke=\"" << t.color
                << "\" stroke-width=\"3\" stroke-linecap=\"round\" d=\"";
            for (const auto& p : t.poses) {
                out << "M" << vp.px(p.x) << " " << vp.py(p.y) << "h0";
            }
            out << "\"/>\n";
            continue;  // no direction markers on raw scatter
        }

        out << "  <polyline fill=\"none\" stroke=\"" << t.color
            << "\" stroke-width=\"2\" stroke-linejoin=\"round\" points=\"";
        for (const auto& p : t.poses) {
            out << vp.px(p.x) << "," << vp.py(p.y) << " ";
        }
        out << "\"/>\n";

        const auto& first = t.poses.front();
        const auto& last  = t.poses.back();
        out << "  <circle cx=\"" << vp.px(first.x) << "\" cy=\"" << vp.py(first.y)
            << "\" r=\"5\" fill=\"" << t.color << "\"/>\n";
        out << "  <circle cx=\"" << vp.px(last.x) << "\" cy=\"" << vp.py(last.y)
            << "\" r=\"5\" fill=\"white\" stroke=\"" << t.color
            << "\" stroke-width=\"2\"/>\n";
    }

    // Legend, stacked in the top-left corner
    double legend_y = 24.0;
    for (const auto& t : trajs) {
        if (t.style == Style::Points) {
            // Three dots, so the swatch reads the same way the data is drawn
            for (int k = 0; k < 3; ++k) {
                out << "  <circle cx=\"" << 20 + k * 10 << "\" cy=\"" << legend_y
                    << "\" r=\"1.5\" fill=\"" << t.color << "\"/>\n";
            }
        } else {
            out << "  <line x1=\"16\" y1=\"" << legend_y << "\" x2=\"44\" y2=\""
                << legend_y << "\" stroke=\"" << t.color << "\" stroke-width=\"3\"/>\n";
        }
        out << "  <text x=\"52\" y=\"" << legend_y + 4
            << "\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#222\">"
            << t.name << " (" << t.poses.size() << " poses)</text>\n";
        legend_y += 20.0;
    }

    // Scale bar: one meter, so drift can be eyeballed in real units
    out << "  <line x1=\"16\" y1=\"" << height - 20 << "\" x2=\"" << 16 + vp.scale
        << "\" y2=\"" << height - 20 << "\" stroke=\"#222\" stroke-width=\"2\"/>\n";
    out << "  <text x=\"16\" y=\"" << height - 26
        << "\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#222\">1 m</text>\n";

    out << "</svg>\n";

    std::cout << "viz: wrote " << path << "\n";
    return true;
}

} // namespace viz

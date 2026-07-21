// run_logger.h -- lightweight per-control-step CSV telemetry logger.
//
// Added for the STLMPC revision simulation campaign (B1-B5). Header-only so it
// can be dropped into any node without touching CMakeLists. A node holds one
// RunLogger, initializes it from rosparams (enable_logging, log_file), and calls
// row({{"name",value},...}) once per control step. The first row establishes the
// CSV header from the supplied key order; every subsequent row must supply the
// same keys in the same order. Per-run identifiers (config, seed, map) are NOT
// stored here -- they are encoded in the log-file name by scripts/run_campaign.py
// and recovered by scripts/aggregate_runs.py, keeping this class trivial.
//
// Nothing is written when logging is disabled, so the logger is safe to leave
// compiled-in for hardware runs.

#ifndef F1TENTH_SIMULATOR_RUN_LOGGER_H
#define F1TENTH_SIMULATOR_RUN_LOGGER_H

#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <iomanip>

class RunLogger {
public:
    RunLogger() : enabled_(false), header_written_(false) {}

    // Call once (e.g. in the node constructor). If enabled and path is
    // non-empty, the file is opened/truncated immediately.
    void init(const std::string& path, bool enabled) {
        enabled_ = enabled && !path.empty();
        header_written_ = false;
        if (enabled_) {
            ofs_.open(path.c_str(), std::ios::out | std::ios::trunc);
            enabled_ = ofs_.is_open();
        }
    }

    bool enabled() const { return enabled_; }

    // Append one row. kv is an ordered list of (column-name, value).
    void row(const std::vector<std::pair<std::string, double> >& kv) {
        if (!enabled_) return;
        if (!header_written_) {
            for (size_t i = 0; i < kv.size(); ++i) {
                ofs_ << kv[i].first;
                if (i + 1 < kv.size()) ofs_ << ",";
            }
            ofs_ << "\n";
            header_written_ = true;
        }
        ofs_ << std::setprecision(9);
        for (size_t i = 0; i < kv.size(); ++i) {
            ofs_ << kv[i].second;
            if (i + 1 < kv.size()) ofs_ << ",";
        }
        ofs_ << "\n";
        ofs_.flush(); // flush each step so a killed/timed-out run still yields data
    }

    ~RunLogger() { if (ofs_.is_open()) ofs_.close(); }

private:
    bool enabled_;
    bool header_written_;
    std::ofstream ofs_;
};

#endif // F1TENTH_SIMULATOR_RUN_LOGGER_H

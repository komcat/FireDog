// Inspection.h : the inspection collector - every inspect_object verdict, kept
// per component (the step's ctx_prefix) until the operator clears it, with the
// statistics the Inspection View shows and the CSV it saves.
//
// Pure C++ (no UI, no hardware). The samples arrive from engine workers through
// Workflow::QueueInspection; App::Update drains them into ONE collector, which
// is UI-thread only like ProductSession. Memory only - a save is an explicit
// CSV. docs/inspection-view-design.md.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace firedog {

// One inspect_object verdict. `found` false = nothing to measure (object not
// found, or no live datum) - counted as a run that did not pass, no numbers.
struct InspectionSample {
    std::string name;     // the component: the step's ctx_prefix
    std::string datum;
    std::string at;       // NowStamp
    std::string serial;   // the open DUT's, else ""
    bool        found  = false;
    bool        judged = false;   // a tolerance was set
    bool        pass   = false;
    std::string why;              // why not a pass
    double lx = 0.0, ly = 0.0;    // datum-local mm
    double dlx = 0.0, dly = 0.0, dr = 0.0;
    double nomX = 0.0, nomY = 0.0;
    double tolX = 0.0, tolY = 0.0, tolR = 0.0;   // 0 = not checked
    bool   hasP  = false;
    double p     = 0.0;
    double score = 0.0;
};

// One metric's statistics over the FOUND samples of a component. Cpk only when
// the metric has a tolerance, n >= 2 and the spread is not zero: dX / dY are
// two-sided (+/-tol around 0), dR one-sided (upper = tol).
struct MetricStats {
    int    n    = 0;
    double last = 0.0, mean = 0.0, min = 0.0, max = 0.0, stdev = 0.0;
    bool   hasCpk = false;
    double cpk    = 0.0;
    double tol    = 0.0;          // the tolerance Cpk was taken against
    bool   oneSided = false;
};

// The metrics a component reports, in the order the view lists them.
enum class InspMetric { Lx, Ly, Dlx, Dly, Dr, P, Score };
const std::vector<InspMetric>& InspMetrics();
const char* InspMetricName(InspMetric m);   // "x (local)", "dX", ...
const char* InspMetricKey(InspMetric m);    // "lx", "dlx", ... (CSV)

class InspectionCollector {
public:
    void Add(const InspectionSample& s);
    void Clear();
    bool Empty() const { return total_ == 0; }
    int  Total() const { return total_; }

    // Components in the order they were first seen.
    const std::vector<std::string>& Components() const { return order_; }
    const std::vector<InspectionSample>& Samples(const std::string& name) const;

    // Per component: runs, found, judged, passed; and a metric's statistics
    // (P only over samples that carry it). Cpk uses the LATEST sample's
    // tolerance - an edited step judges from then on.
    struct Counts { int runs = 0, found = 0, judged = 0, passed = 0; };
    Counts      CountsFor(const std::string& name) const;
    MetricStats StatsFor(const std::string& name, InspMetric m) const;

    // Every sample, one row each, oldest first per component.
    std::string SamplesCsv() const;
    // One row per component (counts, yield) and per component x metric.
    std::string SummaryCsv() const;

private:
    std::map<std::string, std::vector<InspectionSample>> data_;
    std::vector<std::string> order_;
    int total_ = 0;
};

// A name the operator typed, made safe for a file name: letters, digits,
// '-', '_', '.'; anything else becomes '_'; "" stays "".
std::string InspectionFileName(const std::string& typed);

} // namespace firedog

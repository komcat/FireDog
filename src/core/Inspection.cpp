// Inspection.cpp : the inspection collector's statistics and CSV.
// docs/inspection-view-design.md.
#include "core/Inspection.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace firedog {

namespace {

double MetricValue(const InspectionSample& s, InspMetric m)
{
    switch (m) {
    case InspMetric::Lx:    return s.lx;
    case InspMetric::Ly:    return s.ly;
    case InspMetric::Dlx:   return s.dlx;
    case InspMetric::Dly:   return s.dly;
    case InspMetric::Dr:    return s.dr;
    case InspMetric::P:     return s.p;
    case InspMetric::Score: return s.score;
    }
    return 0.0;
}

// A CSV field: quoted when it holds a separator, a quote or a line break.
std::string CsvField(const std::string& v)
{
    if (v.find_first_of(",\"\r\n") == std::string::npos) return v;
    std::string out = "\"";
    for (const char c : v) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

std::string Num(double v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6f", v);
    return buf;
}

} // namespace

const std::vector<InspMetric>& InspMetrics()
{
    static const std::vector<InspMetric> kAll = {InspMetric::Lx,  InspMetric::Ly,
                                                 InspMetric::Dlx, InspMetric::Dly,
                                                 InspMetric::Dr,  InspMetric::P,
                                                 InspMetric::Score};
    return kAll;
}

const char* InspMetricName(InspMetric m)
{
    switch (m) {
    case InspMetric::Lx:    return "x (local)";
    case InspMetric::Ly:    return "y (local)";
    case InspMetric::Dlx:   return "dX from nominal";
    case InspMetric::Dly:   return "dY from nominal";
    case InspMetric::Dr:    return "dR from nominal";
    case InspMetric::P:     return "P (deg)";
    case InspMetric::Score: return "score";
    }
    return "?";
}

const char* InspMetricKey(InspMetric m)
{
    switch (m) {
    case InspMetric::Lx:    return "lx";
    case InspMetric::Ly:    return "ly";
    case InspMetric::Dlx:   return "dlx";
    case InspMetric::Dly:   return "dly";
    case InspMetric::Dr:    return "dr";
    case InspMetric::P:     return "p";
    case InspMetric::Score: return "score";
    }
    return "?";
}

void InspectionCollector::Add(const InspectionSample& s)
{
    auto it = data_.find(s.name);
    if (it == data_.end()) {
        order_.push_back(s.name);
        it = data_.emplace(s.name, std::vector<InspectionSample>{}).first;
    }
    it->second.push_back(s);
    ++total_;
}

void InspectionCollector::Clear()
{
    data_.clear();
    order_.clear();
    total_ = 0;
}

const std::vector<InspectionSample>& InspectionCollector::Samples(const std::string& name) const
{
    static const std::vector<InspectionSample> kNone;
    const auto it = data_.find(name);
    return it == data_.end() ? kNone : it->second;
}

InspectionCollector::Counts InspectionCollector::CountsFor(const std::string& name) const
{
    Counts c;
    for (const InspectionSample& s : Samples(name)) {
        ++c.runs;
        if (s.found) ++c.found;
        if (s.judged) ++c.judged;
        if (s.pass) ++c.passed;
    }
    return c;
}

MetricStats InspectionCollector::StatsFor(const std::string& name, InspMetric m) const
{
    MetricStats st;
    const std::vector<InspectionSample>& all = Samples(name);
    double sum = 0.0;
    const InspectionSample* latest = nullptr;
    for (const InspectionSample& s : all) {
        if (!s.found) continue;
        if (m == InspMetric::P && !s.hasP) continue;
        const double v = MetricValue(s, m);
        if (st.n == 0) { st.min = v; st.max = v; }
        st.min  = std::min(st.min, v);
        st.max  = std::max(st.max, v);
        st.last = v;
        sum += v;
        ++st.n;
        latest = &s;
    }
    if (st.n == 0) return st;
    st.mean = sum / st.n;
    if (st.n >= 2) {
        double ss = 0.0;
        for (const InspectionSample& s : all) {
            if (!s.found || (m == InspMetric::P && !s.hasP)) continue;
            const double d = MetricValue(s, m) - st.mean;
            ss += d * d;
        }
        st.stdev = std::sqrt(ss / (st.n - 1));   // sample standard deviation
    }

    // Cpk against the latest sample's tolerance.
    double tol = 0.0;
    if (m == InspMetric::Dlx) tol = latest->tolX;
    if (m == InspMetric::Dly) tol = latest->tolY;
    if (m == InspMetric::Dr) { tol = latest->tolR; st.oneSided = true; }
    st.tol = tol;
    if (tol > 0.0 && st.n >= 2 && st.stdev > 0.0) {
        st.hasCpk = true;
        st.cpk = st.oneSided ? (tol - st.mean) / (3.0 * st.stdev)
                             : std::min(tol - st.mean, st.mean + tol) / (3.0 * st.stdev);
    }
    return st;
}

std::string InspectionCollector::SamplesCsv() const
{
    std::string out = "time,serial,component,datum,found,judged,pass,lx,ly,nominal_lx,"
                      "nominal_ly,dlx,dly,dr,tol_x_mm,tol_y_mm,tol_r_mm,p,score,why\n";
    for (const std::string& name : order_) {
        for (const InspectionSample& s : Samples(name)) {
            out += CsvField(s.at) + "," + CsvField(s.serial) + "," + CsvField(s.name) + "," +
                   CsvField(s.datum) + "," + (s.found ? "1" : "0") + "," +
                   (s.judged ? "1" : "0") + "," + (s.pass ? "1" : "0") + ",";
            if (s.found)
                out += Num(s.lx) + "," + Num(s.ly) + "," + Num(s.nomX) + "," + Num(s.nomY) + "," +
                       Num(s.dlx) + "," + Num(s.dly) + "," + Num(s.dr) + ",";
            else
                out += ",,,,,,,";
            out += Num(s.tolX) + "," + Num(s.tolY) + "," + Num(s.tolR) + ",";
            out += (s.found && s.hasP ? Num(s.p) : std::string()) + ",";
            out += (s.found ? Num(s.score) : std::string()) + ",";
            out += CsvField(s.why) + "\n";
        }
    }
    return out;
}

std::string InspectionCollector::SummaryCsv() const
{
    std::string out = "component,metric,n,last,mean,min,max,range,stdev,tol,cpk,runs,found,"
                      "judged,passed,yield_pct\n";
    for (const std::string& name : order_) {
        const Counts c = CountsFor(name);
        char yield[32] = "";
        if (c.runs > 0) std::snprintf(yield, sizeof(yield), "%.2f", 100.0 * c.passed / c.runs);
        out += CsvField(name) + ",(all),,,,,,,,,," + std::to_string(c.runs) + "," +
               std::to_string(c.found) + "," + std::to_string(c.judged) + "," +
               std::to_string(c.passed) + "," + yield + "\n";
        for (const InspMetric m : InspMetrics()) {
            const MetricStats st = StatsFor(name, m);
            if (st.n == 0) continue;
            out += CsvField(name) + "," + InspMetricKey(m) + "," + std::to_string(st.n) + "," +
                   Num(st.last) + "," + Num(st.mean) + "," + Num(st.min) + "," + Num(st.max) +
                   "," + Num(st.max - st.min) + "," + (st.n >= 2 ? Num(st.stdev) : std::string()) +
                   "," + (st.tol > 0.0 ? Num(st.tol) : std::string()) + "," +
                   (st.hasCpk ? Num(st.cpk) : std::string()) + ",,,,,\n";
        }
    }
    return out;
}

std::string InspectionFileName(const std::string& typed)
{
    std::string out;
    for (const char c : typed) {
        const unsigned char u = static_cast<unsigned char>(c);
        out += (std::isalnum(u) || c == '-' || c == '_' || c == '.') ? c : '_';
    }
    // No leading dots (hidden / relative-looking names).
    while (!out.empty() && out.front() == '.') out.erase(out.begin());
    return out;
}

} // namespace firedog

#define _CRT_SECURE_NO_WARNINGS 1
// tests/unit/test_field_reference.cpp - the model against the DWD station record of Frankfurt/Main (01420) and the DWD
// AMBAV evapotranspiration of winter wheat (CDC derived_germany/soil/daily, VRWL loam / VRWS sand). The station files are
// not in the repository; the hidden diagnostic reads them from $PS_DWD_DIR (kl/produkt_klima_tag_*.txt, h1420, r1420).
#include "field_test_support.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

using namespace ps;
using namespace ps::field;
namespace tf = ps::test_field;

namespace {
struct Rec { double tn, tx, rain, et0, vrwl, vrws; };
std::map<int, Rec> load_dwd(const std::string& dir) {
    std::map<int, Rec> m;
    {
        std::ifstream f(dir + "/kl/produkt_klima_tag_19350701_20251231_01420.txt");
        std::string l; std::getline(f, l);
        while (std::getline(f, l)) {
            std::stringstream ss(l); std::string t; std::vector<std::string> p;
            while (std::getline(ss, t, ';')) p.push_back(t);
            if (p.size() < 18) continue;
            const int d = std::stoi(p[1]);
            if (d < 19910101) continue;
            Rec r{}; r.rain = std::stod(p[6]); r.tx = std::stod(p[15]); r.tn = std::stod(p[16]);
            m[d] = r;
        }
    }
    for (const char* fn : {"/h1420", "/r1420"}) {
        std::ifstream f(dir + fn);
        std::string l; std::getline(f, l);
        while (std::getline(f, l)) {
            std::stringstream ss(l); std::string t; std::vector<std::string> p;
            while (std::getline(ss, t, ';')) p.push_back(t);
            if (p.size() < 30) continue;
            const int d = std::stoi(p[1]);
            auto it = m.find(d);
            if (it == m.end()) continue;
            it->second.et0 = std::stod(p[24]);
            it->second.vrwl = std::stod(p[29]);
            it->second.vrws = std::stod(p[28]);
        }
    }
    return m;
}
}

TEST_CASE("F1c-D1 diagnostic: model ETa and drainage on the DWD Frankfurt record vs AMBAV", "[.][diag][field]") {
    const char* dir = std::getenv("PS_DWD_DIR");
    if (!dir) { SKIP("PS_DWD_DIR not set"); }
    const std::map<int, Rec> rec = load_dwd(dir);
    for (const SoilDesc* soil : {&tf::loess(), &tf::sandy()}) {
        FieldConfig cfg = tf::one_cell_config(*soil, tf::wheat(), 1, 1, 0.2);
        Field f(cfg);
        double eta = 0, ev = 0, tr = 0, dr = 0, ambav = 0, pr = 0; int yrs = 0; double mE[12]={}, mT[12]={}, mA[12]={}, mP[12]={}, mD[12]={};
        std::int64_t day = 0; int cur_year = 0; double a_eta = 0, a_ev = 0, a_tr = 0, a_dr = 0, a_amb = 0, a_pr = 0;
        for (const auto& [d, r] : rec) {
            const int y = d / 10000, mo = (d / 100) % 100, dd = d % 100;
            if (mo == 2 && dd == 29) continue;
            static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
            const int doy = cum[mo - 1] + dd;
            if (y != cur_year) {
                if (cur_year >= 1993 && cur_year <= 2020) {
                    eta += a_eta; ev += a_ev; tr += a_tr; dr += a_dr; ambav += a_amb; pr += a_pr; ++yrs;
                }
                cur_year = y; a_eta = a_ev = a_tr = a_dr = a_amb = a_pr = 0;
            }
            const double tm = 0.5 * (r.tn + r.tx);
            DailyWeather w = tf::synth(day, r.tn, r.tx, tm > 0 ? r.rain : 0.0, tm > 0 ? 0.0 : r.rain, r.et0);
            w.doy = doy;
            if (doy == 288) f.sow_all(1);
            if (doy == 212 && cur_year > 1991) f.harvest_all();
            f.step_day(w);
            const CellView v = f.view(0);
            a_eta += v.last_e_mm + v.last_tr_mm; a_ev += v.last_e_mm; a_tr += v.last_tr_mm; a_dr += v.last_drain_mm;
            a_amb += (soil == &tf::loess()) ? r.vrwl : r.vrws; a_pr += r.rain;
            if (cur_year >= 1993 && cur_year <= 2020) { mE[mo-1]+=v.last_e_mm; mT[mo-1]+=v.last_tr_mm; mA[mo-1]+=(soil == &tf::loess()) ? r.vrwl : r.vrws; mP[mo-1]+=r.rain; mD[mo-1]+=v.last_drain_mm; }
            ++day;
        }
        std::printf("F1c-D1 %s: %d years  P %.0f  ETa(model) %.0f (E %.0f Tr %.0f)  AMBAV wheat %.0f  drain %.0f\n", soil->name.c_str(), yrs,
                    pr / yrs, eta / yrs, ev / yrs, tr / yrs, ambav / yrs, dr / yrs);
        for (int m = 0; m < 12; ++m) std::printf("   m%02d P %5.1f  E %5.1f Tr %5.1f  ETa %5.1f  AMBAV %5.1f  drain %5.1f\n", m + 1, mP[m]/28, mE[m]/28, mT[m]/28, (mE[m]+mT[m])/28, mA[m]/28, mD[m]/28);
    }
}

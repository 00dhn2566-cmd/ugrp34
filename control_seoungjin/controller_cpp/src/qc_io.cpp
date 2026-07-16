// qc_io.cpp — INTERFACE_SPEC v0.1 파일 계약 구현
// JSON은 외부 의존 없이 필요한 만큼만 파싱 (숫자/문자열/배열/객체/불리언/널).

#include "qc_io.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <ctime>
#include <filesystem>

namespace qcio {

// ---------- 미니 JSON 파서 ----------

namespace mj {

struct Value;
using Arr = std::vector<Value>;
using ObjEntry = std::pair<std::string, Value>;

struct Value {
    enum class T { Null, Bool, Num, Str, Arr, Obj } t = T::Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Value> a;
    std::vector<ObjEntry> o;

    const Value* find(const std::string& key) const {
        if (t != T::Obj) return nullptr;
        for (const auto& kv : o)
            if (kv.first == key) return &kv.second;
        return nullptr;
    }
};

struct Parser {
    const char* p;
    const char* end;
    std::string err;

    void ws() { while (p < end && (*p==' '||*p=='\t'||*p=='\n'||*p=='\r')) ++p; }

    bool fail(const char* m) { if (err.empty()) err = m; return false; }

    bool parse(Value& v) {
        ws();
        if (p >= end) return fail("입력 소진");
        switch (*p) {
        case '{': return obj(v);
        case '[': return arr(v);
        case '"': v.t = Value::T::Str; return str(v.s);
        case 't': if (end-p>=4 && !std::strncmp(p,"true",4))  { v.t=Value::T::Bool; v.b=true;  p+=4; return true; } return fail("true?");
        case 'f': if (end-p>=5 && !std::strncmp(p,"false",5)) { v.t=Value::T::Bool; v.b=false; p+=5; return true; } return fail("false?");
        case 'n': if (end-p>=4 && !std::strncmp(p,"null",4))  { v.t=Value::T::Null; p+=4; return true; } return fail("null?");
        default:  return num(v);
        }
    }

    bool str(std::string& out) {
        ++p; // opening quote
        out.clear();
        while (p < end && *p != '"') {
            if (*p == '\\' && p+1 < end) {
                ++p;
                switch (*p) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'u': // \uXXXX — 이 계약의 키/해시는 ASCII라 원문 유지로 충분
                    out += "\\u"; break;
                default: out += *p; break;
                }
                ++p;
            } else out += *p++;
        }
        if (p >= end) return fail("문자열 미종결");
        ++p;
        return true;
    }

    bool num(Value& v) {
        char* q = nullptr;
        v.n = std::strtod(p, &q);
        if (q == p) return fail("숫자 아님");
        v.t = Value::T::Num;
        p = q;
        return true;
    }

    bool arr(Value& v) {
        v.t = Value::T::Arr;
        ++p; ws();
        if (p < end && *p == ']') { ++p; return true; }
        while (true) {
            v.a.emplace_back();
            if (!parse(v.a.back())) return false;
            ws();
            if (p < end && *p == ',') { ++p; continue; }
            if (p < end && *p == ']') { ++p; return true; }
            return fail("배열 구분자");
        }
    }

    bool obj(Value& v) {
        v.t = Value::T::Obj;
        ++p; ws();
        if (p < end && *p == '}') { ++p; return true; }
        while (true) {
            ws();
            if (p >= end || *p != '"') return fail("키 따옴표");
            std::string key;
            if (!str(key)) return false;
            ws();
            if (p >= end || *p != ':') return fail("콜론");
            ++p;
            v.o.emplace_back(key, Value{});
            if (!parse(v.o.back().second)) return false;
            ws();
            if (p < end && *p == ',') { ++p; continue; }
            if (p < end && *p == '}') { ++p; return true; }
            return fail("객체 구분자");
        }
    }
};

static bool parse_file(const std::string& path, Value& root, std::string& err) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) { err = "파일 열기 실패: " + path; return false; }
    std::string buf;
    char chunk[65536];
    size_t n;
    while ((n = std::fread(chunk, 1, sizeof chunk, f)) > 0) buf.append(chunk, n);
    std::fclose(f);
    Parser ps{buf.data(), buf.data() + buf.size(), {}};
    if (!ps.parse(root)) { err = "JSON 파싱 실패(" + ps.err + "): " + path; return false; }
    return true;
}

} // namespace mj

// ---------- §2 궤적 로드/샘플 ----------

bool load_trajectory(const std::string& path, Trajectory& out, std::string& err) {
    mj::Value root;
    if (!mj::parse_file(path, root, err)) return false;

    const mj::Value* jdt  = root.find("dt");
    const mj::Value* jh   = root.find("trajectory_hash");
    const mj::Value* jt   = root.find("t");
    const mj::Value* jpos = root.find("pos");
    const mj::Value* jyaw = root.find("yaw_rad");
    if (!jt || !jpos || !jyaw) { err = "필수 키 누락 (t/pos/yaw_rad): " + path; return false; }

    out = Trajectory{};
    if (jdt && jdt->t == mj::Value::T::Num) out.dt = jdt->n;
    if (jh && jh->t == mj::Value::T::Str) out.hash = jh->s;

    const size_t N = jt->a.size();
    if (N < 2 || jpos->a.size() != N || jyaw->a.size() != N) {
        err = "배열 길이 불일치/부족: " + path;
        return false;
    }
    out.t.reserve(N); out.px.reserve(N); out.py.reserve(N); out.pz.reserve(N); out.yaw.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        const mj::Value& pi = jpos->a[i];
        if (pi.a.size() != 3) { err = "pos 행이 3성분 아님"; return false; }
        out.t.push_back(jt->a[i].n);
        out.px.push_back(pi.a[0].n);
        out.py.push_back(pi.a[1].n);
        out.pz.push_back(pi.a[2].n);
        out.yaw.push_back(jyaw->a[i].n);
        if (i > 0 && out.t[i] <= out.t[i-1]) { err = "t 비단조 (스펙 §7 규칙5)"; return false; }
    }
    return true;
}

RefSample sample_trajectory(const Trajectory& tr, double tq) {
    RefSample r{};
    const size_t N = tr.t.size();
    auto at = [&](size_t i, int axis) {
        return axis == 0 ? tr.px[i] : (axis == 1 ? tr.py[i] : tr.pz[i]);
    };
    auto interp = [&](double q, int axis) {
        if (q <= tr.t.front()) return at(0, axis);
        if (q >= tr.t.back())  return at(N-1, axis);
        size_t lo = 0, hi = N - 1;
        while (hi - lo > 1) { size_t m = (lo + hi) / 2; (tr.t[m] <= q ? lo : hi) = m; }
        const double w = (q - tr.t[lo]) / (tr.t[hi] - tr.t[lo]);
        return at(lo, axis) + w * (at(hi, axis) - at(lo, axis));
    };
    auto interpYaw = [&](double q) {
        if (q <= tr.t.front()) return tr.yaw.front();
        if (q >= tr.t.back())  return tr.yaw.back();
        size_t lo = 0, hi = N - 1;
        while (hi - lo > 1) { size_t m = (lo + hi) / 2; (tr.t[m] <= q ? lo : hi) = m; }
        const double w = (q - tr.t[lo]) / (tr.t[hi] - tr.t[lo]);
        return tr.yaw[lo] + w * (tr.yaw[hi] - tr.yaw[lo]);
    };

    const double h = tr.dt;   // 후방차분 vel/acc (성형기 원칙 1과 동일 규약)
    for (int ax = 0; ax < 3; ++ax) {
        const double p0 = interp(tq, ax);
        const double pm = interp(tq - h, ax);
        const double pmm = interp(tq - 2*h, ax);
        r.pos[ax] = p0;
        r.vel[ax] = (p0 - pm) / h;
        r.acc[ax] = (p0 - 2*pm + pmm) / (h*h);
    }
    r.yaw = interpYaw(tq);
    return r;
}

// ---------- 타임스탬프 ----------

std::string now_string(bool withMs) {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const std::time_t tt = system_clock::to_time_t(now);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &tt);
#else
    localtime_r(&tt, &tmv);
#endif
    char buf[40];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H-%M-%S", &tmv);   // traj_pipeline.py TS_FMT
    std::string s(buf);
    if (withMs) {
        const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
        char mbuf[8];
        std::snprintf(mbuf, sizeof mbuf, ".%03d", static_cast<int>(ms.count()));
        s += mbuf;
    }
    return s;
}

// ---------- 원자적 쓰기 공통 ----------

static bool atomic_write(const std::string& path, const std::string& body, std::string& err) {
    const std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) { err = "임시파일 열기 실패: " + tmp; return false; }
    const bool ok = std::fwrite(body.data(), 1, body.size(), f) == body.size();
    std::fclose(f);
    if (!ok) { err = "쓰기 실패: " + tmp; return false; }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);   // 기존 파일 교체 (스펙: 반쯤 써진 JSON 방지)
    if (ec) { err = "rename 실패: " + ec.message(); return false; }
    return true;
}

static void vec3_json(std::string& out, const char* key, const double v[3]) {
    char buf[128];
    std::snprintf(buf, sizeof buf, "\"%s\": [%.6f, %.6f, %.6f]", key, v[0], v[1], v[2]);
    out += buf;
}

// ---------- §5 current_state.json ----------

bool write_current_state(const std::string& path, const CurrentState& st, std::string& err) {
    std::string j = "{\n  \"timestamp\": \"" + now_string(true) + "\",\n  ";
    vec3_json(j, "pos", st.pos); j += ",\n  ";
    vec3_json(j, "vel", st.vel); j += ",\n  ";
    vec3_json(j, "acc", st.acc); j += ",\n  ";
    char buf[64];
    std::snprintf(buf, sizeof buf, "\"yaw_rad\": %.6f", st.yaw);
    j += buf;
    j += ",\n  \"ref_state\": { ";
    vec3_json(j, "pos", st.ref.pos); j += ", ";
    vec3_json(j, "vel", st.ref.vel); j += ", ";
    vec3_json(j, "acc", st.ref.acc);
    j += " }\n}\n";
    return atomic_write(path, j, err);
}

// ---------- §3 attitude_feedback ----------

void FlightLogger::push(double time, double pitchRad, double rollRad, double trackErrM) {
    t.push_back(time);
    pitch.push_back(pitchRad);
    roll.push_back(rollRad);
    trackErr.push_back(trackErrM);
}

FlightLogger::Feedback FlightLogger::analyze() const {
    Feedback fb{};
    if (t.size() < 10 || tArrive <= 0) return fb;
    const double rad2deg = 180.0 / 3.14159265358979323846;

    // 구간 분리
    std::vector<size_t> mov, tail;
    for (size_t i = 0; i < t.size(); ++i)
        (t[i] < tArrive ? mov : tail).push_back(i);
    if (tail.size() < 10 || mov.size() < 10) return fb;

    auto rms = [&](const std::vector<double>& x, const std::vector<size_t>& idx) {
        double m = 0;
        for (size_t i : idx) m += x[i];
        m /= double(idx.size());
        double s = 0;
        for (size_t i : idx) s += (x[i]-m)*(x[i]-m);
        return std::sqrt(s / double(idx.size()));
    };

    // moving 지표
    double pk = 0, te2 = 0;
    for (size_t i : mov) {
        pk = std::max(pk, std::max(std::fabs(pitch[i]), std::fabs(roll[i])));
        te2 += trackErr[i]*trackErr[i];
    }
    fb.movingAttPeakDeg  = pk * rad2deg;
    fb.movingTrackRmsCm  = std::sqrt(te2 / double(mov.size())) * 100.0;

    // tail 지표 (지터 본체)
    fb.tailPitchRmsDeg = rms(pitch, tail) * rad2deg;
    fb.tailRollRmsDeg  = rms(roll,  tail) * rad2deg;

    // 영교차 주파수 (pitch, 평균 제거)
    double mMean = 0;
    for (size_t i : tail) mMean += pitch[i];
    mMean /= double(tail.size());
    int zc = 0;
    for (size_t k = 1; k < tail.size(); ++k) {
        const double a = pitch[tail[k-1]] - mMean, b = pitch[tail[k]] - mMean;
        if ((a < 0 && b >= 0) || (a > 0 && b <= 0)) ++zc;
    }
    const double span = t[tail.back()] - t[tail.front()];
    fb.modeFreqHz = span > 0 ? (0.5 * zc) / span : 0;

    // 고정 주파수 사인 피팅 (최소제곱, analyze_flight_log.py 대응) — 카운터스윙 훅용
    fb.tRefS = t[tail.front()];
    if (fb.modeFreqHz > 0.05) {
        const double w = 2 * 3.14159265358979323846 * fb.modeFreqHz;
        double Scc = 0, Sss = 0, Scs = 0, Syc = 0, Sys = 0;
        for (size_t i : tail) {
            const double ph = w * (t[i] - fb.tRefS);
            const double c = std::cos(ph), s = std::sin(ph);
            const double y = pitch[i] - mMean;
            Scc += c*c; Sss += s*s; Scs += c*s; Syc += y*c; Sys += y*s;
        }
        const double det = Scc*Sss - Scs*Scs;
        if (std::fabs(det) > 1e-12) {
            const double A = ( Sss*Syc - Scs*Sys) / det;   // cos 계수
            const double B = (-Scs*Syc + Scc*Sys) / det;   // sin 계수
            fb.ampDeg  = std::sqrt(A*A + B*B) * rad2deg;
            fb.phaseRad = std::atan2(A, B);                 // y ≈ amp·sin(w·(t−t0)+phase)
        }
    }

    // 유효성 게이트 (PIPELINE_STATUS 교훈: 발산 비행의 tail은 학습 오염)
    fb.valid = fb.movingTrackRmsCm <= 30.0;
    return fb;
}

bool write_attitude_feedback(const std::string& path, const FlightLogger::Feedback& fb,
                             const std::string& trajectoryHash, std::string& err) {
    if (!fb.valid) { err = "유효성 게이트 불합격 (추종 RMS>30cm) — 기록 거부"; return false; }
    const std::string now = now_string(false);
    char buf[1024];
    std::snprintf(buf, sizeof buf,
        "{\n"
        "  \"flight_id\": \"%s\",\n"
        "  \"written_at\": \"%s\",\n"
        "  \"used\": false,\n"
        "  \"trajectory_hash\": \"%s\",\n"
        "  \"mode_freq_hz\": %.4f,\n"
        "  \"tail\": {\n"
        "    \"pitch_rms_deg\": %.4f, \"roll_rms_deg\": %.4f,\n"
        "    \"amp_deg\": %.4f, \"phase_rad\": %.4f, \"t_ref_s\": %.4f\n"
        "  },\n"
        "  \"moving\": { \"att_peak_deg\": %.3f, \"track_rms_cm\": %.3f },\n"
        "  \"k_est\": { \"kthrust\": null, \"kdrag\": null, \"confidence\": 0.0 }\n"
        "}\n",
        now.c_str(), now.c_str(), trajectoryHash.c_str(),
        fb.modeFreqHz, fb.tailPitchRmsDeg, fb.tailRollRmsDeg,
        fb.ampDeg, fb.phaseRad, fb.tRefS,
        fb.movingAttPeakDeg, fb.movingTrackRmsCm);
    return atomic_write(path, buf, err);
}

} // namespace qcio

#include "ppk_core.h"

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" {
#include "rtklib.h"
#include "gpt3.h"
}

namespace {

// GLONASS is converted and solved only when the caller selects it. Each GLONASS satellite has
// its own frequency, and receivers of different makes carry uncalibrated inter-frequency biases;
// even with GLONASS AR off those biases can skew the float solution enough that the other
// systems fail to fix (CHC T7R base + Unicore rover: 0% fixed with GLONASS, 94% without), so the
// app leaves it unselected by default.

// Mirrors MAXINFILE in postpos.c (not exported by rtklib.h).
constexpr int kMaxInputFiles = 1000;

std::atomic<bool> g_cancel{false};
ppk_message_fn g_handler = nullptr;
void* g_handler_context = nullptr;
std::string g_last_message;

gtime_t toGtime(double seconds) {
  gtime_t t = {0, 0.0};
  if (seconds <= 0.0) return t;
  t.time = (time_t)std::floor(seconds);
  t.sec = seconds - (double)t.time;
  return t;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Callbacks RTKLIB expects the host application to provide (declared in rtklib.h).
// ---------------------------------------------------------------------------------------------

extern "C" int showmsg(const char* format, ...) {
  char buff[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(buff, sizeof(buff), format, args);
  va_end(args);

  if (buff[0] != '\0') {
    g_last_message = buff;
    if (g_handler != nullptr) g_handler(g_handler_context, buff);
  }
  // A non-zero return tells RTKLIB to abort.
  return g_cancel ? 1 : 0;
}

extern "C" void settspan(gtime_t ts, gtime_t te) {}

extern "C" void settime(gtime_t time) {}

// ---------------------------------------------------------------------------------------------

void ppk_set_message_handler(ppk_message_fn handler, void* context) {
  g_handler = handler;
  g_handler_context = context;
}

void ppk_cancel(void) {
  g_cancel = true;
}

void ppk_reset_cancel(void) {
  g_cancel = false;
  g_last_message.clear();
}

const char* ppk_last_message(void) {
  return g_last_message.c_str();
}

namespace {

/** RINEX 3.04 conversion options shared by the receiver formats. */
void initRinexOptions(rnxopt_t* opt) {
  memset(opt, 0, sizeof(*opt));
  opt->rnxver = 304;
  opt->obstype = OBSTYPE_PR | OBSTYPE_CP | OBSTYPE_DOP | OBSTYPE_SNR;
  opt->freqtype = FREQTYPE_ALL;
  opt->navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_SBS | SYS_CMP | SYS_IRN;
  opt->ttol = 0.005;
  // autopos = 0 lets convrnx fill APPROX POSITION XYZ: from RTCM 1005/1006 when the stream
  // has them, otherwise from a single-point solution. The base position may come from there.
  opt->autopos = 0;
  opt->outiono = 1;
  opt->outtime = 1;
  opt->outleaps = 1;
  for (int i = 0; i < RNX_NUMSYS; i++) {
    for (int j = 0; j < MAXCODE; j++) opt->mask[i][j] = '1';
    opt->mask[i][MAXCODE] = '\0';
  }
  strcpy(opt->prog, "AlbaNav");
}

/** Runs convrnx() with ofile[0] = OBS and ofile[1] = mixed NAV (RINEX 3, sep_nav = 0). */
int convertToRinex(int format, rnxopt_t* opt, const char* input, const char* obs_out,
                   const char* nav_out) {
  char outputs[9][1024];
  char* ofile[9];
  memset(outputs, 0, sizeof(outputs));
  snprintf(outputs[0], sizeof(outputs[0]), "%s", obs_out ? obs_out : "");
  snprintf(outputs[1], sizeof(outputs[1]), "%s", nav_out ? nav_out : "");
  for (int i = 0; i < 9; i++) ofile[i] = outputs[i];
  return convrnx(format, opt, input, ofile);
}

}  // namespace

int ppk_convert_rtcm3(const char* input, const char* obs_out, const char* nav_out,
                      double approx_time) {
  static rnxopt_t opt;
  initRinexOptions(&opt);
  // MSM and SSR messages carry only the time of week (GLONASS: time of day); RTKLIB resolves
  // the week and day from this approximate time, which must lie within 12 hours of the data.
  opt.trtcm = toGtime(approx_time);
  if (opt.trtcm.time == 0) {
    showmsg("error : no approximate time for RTCM3 conversion");
    return 0;
  }
  // Receiver and antenna fields stay empty: convrnx fills them from RTCM 1033 when present.
  return convertToRinex(STRFMT_RTCM3, &opt, input, obs_out, nav_out);
}

int ppk_convert_unicore(const char* input, const char* obs_out, const char* nav_out) {
  // rnxopt_t is large; keep it off the (small) worker thread stack.
  static rnxopt_t opt;
  initRinexOptions(&opt);
  strcpy(opt.rec[1], "UNICORE");
  return convertToRinex(STRFMT_UNICORE, &opt, input, obs_out, nav_out);
}

int ppk_postpos(const char** files, int n, const ppk_options* o, const char* out_file) {
  if (n < 2 || n > kMaxInputFiles) {
    showmsg("error : need rover obs, base obs and nav files (got %d)", n);
    return -1;
  }

  static prcopt_t popt;
  static solopt_t sopt;
  static filopt_t fopt;
  popt = prcopt_default;
  sopt = solopt_default;
  memset(&fopt, 0, sizeof(fopt));

  popt.mode = o->mode;
  popt.soltype = o->soltype;
  popt.nf = o->nf;
  popt.navsys = o->navsys;
  if (popt.navsys == 0) {
    showmsg("error : no satellite system selected");
    return -1;
  }
  popt.elmin = o->elmin_deg * D2R;
  popt.modear = o->modear;
  popt.glomodear = o->glomodear;
  popt.bdsmodear = o->bdsmodear;
  popt.thresar[0] = o->ar_ratio;
  popt.refpos = o->refpos;
  if (o->refpos == POSOPT_POS_XYZ) {
    if (norm(o->rb, 3) < 6.0e6) {
      showmsg("error : invalid base position");
      return -1;
    }
    for (int i = 0; i < 3; i++) popt.rb[i] = o->rb[i];
  }
  // Precise orbits/clocks come from the SP3/CLK files among the inputs (readpreceph()), SSR
  // corrections from a ".rtcm3" input (SSRAPC: corrections refer to the antenna phase centre,
  // SSRCOM: to the centre of mass). Broadcast navigation files are still required:
  // readobsnav() refuses to run without them, and SSR corrects the broadcast orbits.
  switch (o->sateph) {
    case EPHOPT_PREC:
    case EPHOPT_SSRAPC:
    case EPHOPT_SSRCOM:
      popt.sateph = o->sateph;
      break;
    default:
      popt.sateph = EPHOPT_BRDC;
      break;
  }
  // postpos() applies SSR corrections in the forward pass only: a backward pass would run
  // with the corrections of the last epoch and reject every satellite as too old.
  bool ssr = popt.sateph == EPHOPT_SSRAPC || popt.sateph == EPHOPT_SSRCOM;
  if (ssr && popt.soltype != SOLTYPE_FORWARD) {
    showmsg("warning : SSR corrections are forward only, solution direction set to forward");
    popt.soltype = SOLTYPE_FORWARD;
  }
  bool ppp = o->mode == PMODE_PPP_KINEMA || o->mode == PMODE_PPP_STATIC;
  bool is_static = o->mode == PMODE_STATIC || o->mode == PMODE_PPP_STATIC;
  if (ppp) {
    // PPP has no base to cancel the ionosphere and troposphere: use the dual-frequency
    // ionosphere-free combination, and always estimate the zenith delay (the model alone
    // leaves decimetres). Gradients only when asked for, on static data.
    popt.ionoopt = IONOOPT_IFLC;
    popt.tropopt = is_static && o->tropopt == TROPOPT_ESTG ? TROPOPT_ESTG : TROPOPT_EST;
    if (o->modear == ARMODE_PPPAR) {
      // PPP-AR (ppp_ar.c) needs the uncombined model: L1 and L2 phase each with its own
      // ambiguity state and a slant ionosphere state per satellite, with the satellite code and
      // phase biases removed by the absolute OSB of the bias file. The iono-free combination
      // mixes the two ambiguities into a non-integer one.
      popt.ionoopt = IONOOPT_EST;
      popt.armaxiter = 10;     // partial AR: drop up to this many satellites
      popt.elmaskar = 15.0 * D2R;
    }
    // pppos() re-initializes the receiver clock every epoch from the single-point solution,
    // which pntpos() computes with L1 and the broadcast (Klobuchar) ionosphere. Under a strong
    // ionosphere that clock is off by 10-20 m, common to all satellites, and the default 5 m
    // pre-fit test then rejects every carrier phase: the filter runs on code only (metre-level
    // sigmas, few satellites). The common offset is absorbed by the clock state in the update;
    // single bad observations are still caught by the post-fit residual test. (Running
    // pntpos() with the iono-free combination instead was tried: noisier clock, worse result.)
    popt.maxinno[0] = 100.0;
    // PPP-specific models, all off by default in prcopt_default: satellite antenna PCV (needs an
    // ANTEX input), receiver antenna PCV, phase wind-up, and exclusion of eclipsing satellites.
    // Without wind-up the carrier phase drifts by up to a cycle as the satellite yaws, which
    // each analysis center's clocks absorb differently.
    popt.posopt[0] = 1;
    popt.posopt[1] = 1;
    popt.posopt[2] = 1;
    popt.posopt[3] = 1;
  } else {
    popt.ionoopt = IONOOPT_BRDC;
    // ZTD (+gradient) estimation only makes sense for static data; kinematic stays on the model.
    popt.tropopt = TROPOPT_SAAS;
    if (is_static && (o->tropopt == TROPOPT_EST || o->tropopt == TROPOPT_ESTG)) {
      popt.tropopt = o->tropopt;
    }
  }
  popt.dynamics = o->mode == PMODE_KINEMA || o->mode == PMODE_PPP_KINEMA ? 1 : 0;
  popt.tidecorr = o->tidecorr;
  // "*": take antenna type and ANTENNA: DELTA H/E/N from the RINEX headers.
  strcpy(popt.anttype[0], "*");
  strcpy(popt.anttype[1], "*");
  // A base antenna height of our own replaces the header's: a type other than "*" keeps
  // setpcv() from overwriting antdel[1]. A single-point average is already the antenna
  // position, so it gets no offset at all.
  if (o->refpos == POSOPT_SINGLE || !std::isnan(o->base_ant_h)) {
    popt.anttype[1][0] = '\0';
    popt.antdel[1][0] = 0.0;
    popt.antdel[1][1] = 0.0;
    popt.antdel[1][2] = o->refpos == POSOPT_SINGLE ? 0.0 : o->base_ant_h;
  }

  sopt.posf = SOLF_LLH;
  sopt.times = TIMES_GPST;
  sopt.timef = 1;
  sopt.timeu = 3;
  sopt.outhead = 1;
  sopt.outopt = 1;
  strcpy(sopt.prog, "AlbaNav");

  // Each input argument gets its own index; readobsnav() starts at receiver 1 and moves to
  // the next receiver when a new index added observations. So files[0] is the rover, files[1]
  // (all files its wildcard matches) the base, and nav files add no observations.
  gtime_t ts = toGtime(o->ts);
  gtime_t te = toGtime(o->te);
  // An ".erp" input goes to fopt.eop (earth rotation parameters) instead of the file list.
  static const char* inputs[kMaxInputFiles];
  int count = 0;
  bool gpt3_grid = false;
  for (int i = 0; i < n; i++) {
    const char* ext = strrchr(files[i], '.');
    if (ext && (strcmp(ext, ".atx") == 0 || strcmp(ext, ".ATX") == 0)) {
      // ANTEX antenna model: satellite PCO/PCV (precise orbits refer to the satellite's centre
      // of mass, the clocks to its antenna phase centre) and receiver antennas by type.
      // Several files are merged (postpos.c readpcvs()); for an antenna in more than one, the
      // file given first wins.
      size_t used = strlen(fopt.satantp);
      if (used + strlen(files[i]) + 2 > sizeof(fopt.satantp)) {
        showmsg("warning : too many antenna files, %s ignored", files[i]);
        continue;
      }
      if (used > 0) strcat(fopt.satantp, "\n");
      strcat(fopt.satantp, files[i]);
      snprintf(fopt.rcvantp, sizeof(fopt.rcvantp), "%s", fopt.satantp);
      continue;
    }
    if (ext && strcmp(ext, ".grd") == 0) {
      // GPT3 1-degree grid (gpt3_1.grd): VMF3 mapping functions and GPT3 a priori zenith
      // delays replace NMF and the standard atmosphere (see gpt3.h).
      if (gpt3_load(files[i])) {
        gpt3_grid = true;
      } else {
        showmsg("warning : cannot read GPT3 grid %s, ignored", files[i]);
      }
      continue;
    }
    if (ext && strcmp(ext, ".sec") == 0) {
      // Leap second table (leap.sec), replacing the one compiled into rtkcmn.c.
      if (!read_leaps(files[i])) showmsg("warning : cannot read leap seconds %s, ignored", files[i]);
      continue;
    }
    if (ext && (strcmp(ext, ".erp") == 0 || strcmp(ext, ".ERP") == 0)) {
      // An unreadable ERP file would make postpos() report "error : no erp data", which reads
      // as a failed run; check it here and go on without it instead.
      erp_t erp = {0};
      if (readerp(files[i], &erp) && erp.n > 0) {
        snprintf(fopt.eop, sizeof(fopt.eop), "%s", files[i]);
      } else {
        showmsg("warning : no erp data in %s, ignored", files[i]);
      }
      free(erp.data);
      continue;
    }
    inputs[count++] = files[i];
  }
  // The grid stays loaded between runs (it takes a while to parse); a run without it among the
  // inputs goes back to RTKLIB's own models.
  if (!gpt3_grid) gpt3_free();
  // PPP has no base observation file.
  if (count < (ppp ? 2 : 3)) {
    showmsg(ppp ? "error : need rover obs and nav files (got %d)"
                : "error : need rover obs, base obs and nav files (got %d)",
            count);
    return -1;
  }
  int stat = postpos(ts, te, 0.0, 0.0, &popt, &sopt, &fopt, inputs, count, out_file, "", "");
  if (stat == 1) return 1;
  if (stat < 0) return -1;
  // postpos() also returns 0 when reading the inputs failed; that leaves an "error" message.
  if (g_last_message.rfind("error", 0) == 0) return -1;
  return 0;
}

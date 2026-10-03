// PPK helpers on top of RTKLIB demo5, free of JNI so they can also be built and tested on a
// desktop host. RTKLIB keeps processing state in globals: callers must not run these
// functions concurrently.

#ifndef ALBANAV_PPK_CORE_H
#define ALBANAV_PPK_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Receives progress and error text that RTKLIB reports through showmsg(). */
typedef void (*ppk_message_fn)(void* context, const char* message);

void ppk_set_message_handler(ppk_message_fn handler, void* context);

/** Requests that the running conversion or solution stops at the next epoch. */
void ppk_cancel(void);

/** Clears the cancel request; called at the start of every operation. */
void ppk_reset_cancel(void);

/** Last non-empty message RTKLIB reported. */
const char* ppk_last_message(void);

/**
 * Converts a Unicore binary capture (AA 44 B5 frames: OBSVM + *EPH) to RINEX 3.04.
 * An empty or null output path skips that file.
 *
 * @return 1 on success, 0 on error, -1 when cancelled
 */
int ppk_convert_unicore(const char* input, const char* obs_out, const char* nav_out);

/**
 * Converts an RTCM3 log (MSM or legacy observations, 1005/1006, ephemerides) to RINEX 3.04.
 *
 * @param approx_time approximate GPST of the data as seconds since 1970 (GPST calendar), within
 *     12 hours; RTCM3 observations carry only the time of week (GLONASS: of day)
 * @return 1 on success, 0 on error, -1 when cancelled
 */
int ppk_convert_rtcm3(const char* input, const char* obs_out, const char* nav_out,
                      double approx_time);

typedef struct {
  int mode;            /* PMODE_KINEMA or PMODE_STATIC */
  int soltype;         /* 0: forward, 1: backward, 2: combined */
  int nf;              /* number of frequencies */
  int navsys;          /* SYS_* mask */
  double elmin_deg;    /* elevation mask */
  int modear;          /* ARMODE_* */
  int glomodear;       /* GLONASS AR mode */
  int bdsmodear;       /* BeiDou AR (0/1) */
  double ar_ratio;     /* AR validation threshold */
  int refpos;          /* POSOPT_RINEX, POSOPT_SINGLE or POSOPT_POS_XYZ */
  double rb[3];        /* base marker position (ecef, m) for POSOPT_POS_XYZ */
  double base_ant_h;   /* base antenna height (m); NaN: from the RINEX header */
  double ts;           /* start time, GPST calendar as seconds since 1970 (0: all) */
  double te;           /* end time, same scale (0: all) */
  int tropopt;         /* TROPOPT_SAAS, TROPOPT_EST or TROPOPT_ESTG */
  int sateph;          /* EPHOPT_BRDC, EPHOPT_PREC (SP3/CLK among the input files) or
                          EPHOPT_SSRAPC (a ".rtcm3" SSR log among them; forces forward) */
  int tidecorr;        /* earth tide correction: 0 off, 1 solid earth tide */
} ppk_options;

/**
 * Runs RTKLIB postpos(). files[0] is the rover RINEX obs file, files[1] the base obs file
 * (may contain a '*' wildcard so several base files act as one receiver), and the rest are
 * navigation files (and, with EPHOPT_PREC, SP3 orbit and RINEX clock files; with EPHOPT_SSRAPC,
 * one ".rtcm3" SSR correction log). A file ending in
 * ".erp" is not an observation input: it is read as earth rotation parameters. Writes an RTKLIB .pos file (lat/lon/height, GPST).
 *
 * @return 0 on success, 1 when cancelled, -1 on error (see ppk_last_message())
 */
int ppk_postpos(const char** files, int n, const ppk_options* options, const char* out_file);

#ifdef __cplusplus
}
#endif

#endif  // ALBANAV_PPK_CORE_H

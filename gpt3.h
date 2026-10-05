/*
 * GPT3 empirical troposphere model (1 degree grid) and VMF3 empirical mapping functions,
 * ported from the TU Wien reference code (https://vmf.geo.tuwien.ac.at/codes/: gpt3_1.f90,
 * vmf3_ht.f90, saasthyd.f, asknewet.f). Reference: Landskron and Boehm (2018), "VMF3/GPT3:
 * refined discrete and empirical troposphere mapping functions", J Geod 92:349-360.
 *
 * Once gpt3_load() has read gpt3_1.grd, RTKLIB's tropmapf() returns the VMF3 mapping functions
 * instead of NMF, and the PPP filter takes its a priori hydrostatic and wet zenith delays from
 * GPT3 instead of the standard atmosphere. Without a grid everything stays as in RTKLIB.
 */
#ifndef ALBANAV_GPT3_H
#define ALBANAV_GPT3_H

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Reads gpt3_1.grd (64800 rows of 1 x 1 degree cells). A grid that is already loaded from the
 * same path is kept.
 *
 * @return 1 on success, 0 on error (the previous grid, if any, is dropped)
 */
int gpt3_load(const char *file);

/** Releases the grid; tropmapf() falls back to NMF. */
void gpt3_free(void);

/** 1 when a grid is loaded. */
int gpt3_loaded(void);

/**
 * VMF3 hydrostatic and wet mapping functions with GPT3 a coefficients.
 *
 * @param pos receiver position {lat,lon,h} (rad, m, ellipsoidal height)
 * @param azel azimuth/elevation (rad)
 * @param mfh hydrostatic mapping function (O)
 * @param mfw wet mapping function (O, may be NULL)
 * @return 1 on success, 0 without a grid (outputs untouched)
 */
int gpt3_mapf(gtime_t time, const double *pos, const double *azel, double *mfh, double *mfw);

/**
 * Zenith hydrostatic delay (Saastamoinen with GPT3 pressure) and zenith wet delay (Askne and
 * Nordius with GPT3 water vapour pressure, mean temperature and decrease factor).
 *
 * @return 1 on success, 0 without a grid (outputs untouched)
 */
int gpt3_zenith(gtime_t time, const double *pos, double *zhd, double *zwd);

#ifdef __cplusplus
}
#endif

#endif /* ALBANAV_GPT3_H */

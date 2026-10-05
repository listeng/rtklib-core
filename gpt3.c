/*
 * GPT3 (1 degree grid) and VMF3 empirical mapping functions, see gpt3.h.
 *
 * Ported from the TU Wien reference code gpt3_1.f90, vmf3_ht.f90, saasthyd.f and asknewet.f
 * (https://vmf.geo.tuwien.ac.at/codes/). Only what positioning needs is kept: pressure,
 * temperature, water vapour, the mapping function coefficients ah/aw and the geoid
 * undulation. The gradients of the grid are not read.
 */
#include "gpt3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vmf3_coef.h"

#define GPT3_CELLS 64800 /* 180 x 360 cells of 1 degree, centres at 89.5..-89.5, 0.5..359.5 */
#define GPT3_LINE  2048

/* Seasonal coefficients of one quantity: mean, annual cos/sin, semi-annual cos/sin. */
typedef struct {
    double p[5];   /* pressure (Pa) */
    double T[5];   /* temperature (K) */
    double Q[5];   /* specific humidity (kg/kg) */
    double ah[5];  /* hydrostatic mapping function coefficient */
    double aw[5];  /* wet mapping function coefficient */
    double la[5];  /* water vapour decrease factor */
    double Tm[5];  /* mean temperature of the water vapour (K) */
    double undu;   /* geoid undulation (m) */
    double Hs;     /* orthometric height of the cell (m) */
} gpt3_cell_t;

/* Interpolated GPT3 values at a station. */
typedef struct {
    double p;      /* pressure (hPa) */
    double e;      /* water vapour pressure (hPa) */
    double Tm;     /* mean temperature (K) */
    double la;     /* water vapour decrease factor */
    double ah, aw; /* VMF3 a coefficients */
} gpt3_val_t;

static gpt3_cell_t *grid = NULL;
static char grid_file[1024] = "";

/* Last station evaluated by gpt3_mapf()/gpt3_zenith(): the filter asks for the same receiver
 * position many times per epoch, and the empirical values only change with the season. */
static struct {
    int valid;
    gtime_t time;
    double pos[3];
    gpt3_val_t val;
    double bh, bw, ch, cw;
} cache;

/* Reads n numbers from *p; returns 0 when the line ends early. */
static int read_values(char **p, double *out, int n)
{
    char *end;
    int i;

    for (i = 0; i < n; i++) {
        out[i] = strtod(*p, &end);
        if (end == *p) return 0;
        *p = end;
    }
    return 1;
}

extern int gpt3_load(const char *file)
{
    FILE *fp;
    char line[GPT3_LINE], *p;
    double latlon[2], skip[20];
    gpt3_cell_t *cells;
    int i, j, ok = 1;

    if (grid && !strcmp(grid_file, file)) return 1;
    gpt3_free();

    if (!(fp = fopen(file, "r"))) return 0;
    if (!(cells = (gpt3_cell_t *)malloc(sizeof(gpt3_cell_t) * GPT3_CELLS))) {
        fclose(fp);
        return 0;
    }
    /* header line: "%  lat    lon   p:a0 ..." */
    if (!fgets(line, sizeof(line), fp)) ok = 0;

    for (i = 0; ok && i < GPT3_CELLS; i++) {
        gpt3_cell_t *c = cells + i;
        if (!fgets(line, sizeof(line), fp)) {
            ok = 0;
            break;
        }
        p = line;
        ok = read_values(&p, latlon, 2) &&
             read_values(&p, c->p, 5) &&
             read_values(&p, c->T, 5) &&
             read_values(&p, c->Q, 5) &&
             read_values(&p, skip, 5) &&    /* dT: lapse rate, not needed for the delays */
             read_values(&p, &c->undu, 1) &&
             read_values(&p, &c->Hs, 1) &&
             read_values(&p, c->ah, 5) &&
             read_values(&p, c->aw, 5) &&
             read_values(&p, c->la, 5) &&
             read_values(&p, c->Tm, 5) &&
             read_values(&p, skip, 20); /* gradients Gn_h, Ge_h, Gn_w, Ge_w */

        /* the interpolation below relies on the row order of the grid; the last column of
         * each row is written as longitude -0.5 rather than 359.5 */
        if (ok && (fabs(latlon[0] - (89.5 - i / 360)) > 1E-6 ||
                   fabs(fmod(latlon[1] + 360.0, 360.0) - (0.5 + i % 360)) > 1E-6)) {
            ok = 0;
        }
        for (j = 0; ok && j < 5; j++) {
            c->Q[j] /= 1000.0;
            c->ah[j] /= 1000.0;
            c->aw[j] /= 1000.0;
        }
    }
    fclose(fp);

    if (!ok) {
        free(cells);
        return 0;
    }
    grid = cells;
    snprintf(grid_file, sizeof(grid_file), "%s", file);
    cache.valid = 0;
    return 1;
}

extern void gpt3_free(void)
{
    free(grid);
    grid = NULL;
    grid_file[0] = '\0';
    cache.valid = 0;
}

extern int gpt3_loaded(void)
{
    return grid != NULL;
}

/* Seasonal model a0 + A1 cos(2 pi t) + B1 sin(2 pi t) + A2 cos(4 pi t) + B2 sin(4 pi t). */
static double seasonal(const double *c, const double *harm)
{
    return c[0] + c[1] * harm[0] + c[2] * harm[1] + c[3] * harm[2] + c[4] * harm[3];
}

/* cos(2 pi t), sin(2 pi t), cos(4 pi t), sin(4 pi t) with t the fraction of the year. */
static void harmonics(gtime_t time, double *harm)
{
    double doy = time2doy(time);

    harm[0] = cos(doy / 365.25 * 2.0 * PI);
    harm[1] = sin(doy / 365.25 * 2.0 * PI);
    harm[2] = cos(doy / 365.25 * 4.0 * PI);
    harm[3] = sin(doy / 365.25 * 4.0 * PI);
}

/* GPT3 values of one grid cell at ellipsoidal height h. */
static void cell_values(const gpt3_cell_t *c, const double *harm, double h, double *p,
                        double *e, double *Tm, double *la, double *ah, double *aw,
                        double *undu)
{
    const double gm = 9.80665, dMtr = 28.965E-3, Rg = 8.3143;
    double T0, p0, Q, redh, Tv, cst, e0;

    *undu = c->undu;
    redh = h - c->undu - c->Hs; /* height above the cell's mean orography */

    T0 = seasonal(c->T, harm);
    p0 = seasonal(c->p, harm);
    Q = seasonal(c->Q, harm);

    Tv = T0 * (1.0 + 0.6077 * Q);
    cst = gm * dMtr / (Rg * Tv);
    *p = p0 * exp(-cst * redh) / 100.0;

    *ah = seasonal(c->ah, harm);
    *aw = seasonal(c->aw, harm);
    *la = seasonal(c->la, harm);
    *Tm = seasonal(c->Tm, harm);

    /* water vapour pressure on the grid, then at station height (Askne and Nordius, 1987) */
    e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0;
    *e = e0 * pow(100.0 * *p / p0, *la + 1.0);
}

/* gpt3_1.f90 with it = 0 (seasonal terms on), for one station. */
static void gpt3_values(gtime_t time, const double *pos, gpt3_val_t *val)
{
    double harm[4], plon, ppod, diffpod, difflon;
    double p[4], e[4], Tm[4], la[4], ah[4], aw[4], undu[4];
    double dnpod1, dnpod2, dnlon1, dnlon2;
    int ipod, ilon, ipod1, ilon1, indx[4], i;

    harmonics(time, harm);

    plon = (pos[1] < 0.0 ? pos[1] + 2.0 * PI : pos[1]) * R2D;
    ppod = (PI / 2.0 - pos[0]) * R2D; /* polar distance (deg) */

    ipod = (int)floor(ppod + 1.0);
    ilon = (int)floor(plon + 1.0);
    diffpod = ppod - (ipod - 0.5);
    difflon = plon - (ilon - 0.5);
    if (ipod == 181) ipod = 180;
    if (ilon == 361) ilon = 1;
    if (ilon == 0) ilon = 360;

    indx[0] = (ipod - 1) * 360 + ilon - 1;

    if (ppod <= 0.5 || ppod >= 179.5) {
        /* near the poles: nearest cell */
        cell_values(grid + indx[0], harm, pos[2], &val->p, &val->e, &val->Tm, &val->la,
                    &val->ah, &val->aw, undu);
        return;
    }
    /* bilinear interpolation between the four surrounding cell centres */
    ipod1 = ipod + (diffpod >= 0.0 ? 1 : -1);
    ilon1 = ilon + (difflon >= 0.0 ? 1 : -1);
    if (ilon1 == 361) ilon1 = 1;
    if (ilon1 == 0) ilon1 = 360;

    indx[1] = (ipod1 - 1) * 360 + ilon - 1;  /* same longitude */
    indx[2] = (ipod - 1) * 360 + ilon1 - 1;  /* same polar distance */
    indx[3] = (ipod1 - 1) * 360 + ilon1 - 1; /* diagonal */

    for (i = 0; i < 4; i++) {
        cell_values(grid + indx[i], harm, pos[2], p + i, e + i, Tm + i, la + i, ah + i, aw + i,
                    undu + i);
    }
    dnpod1 = fabs(diffpod);
    dnpod2 = 1.0 - dnpod1;
    dnlon1 = fabs(difflon);
    dnlon2 = 1.0 - dnlon1;

#define BILINEAR(v) (dnlon2 * (dnpod2 * (v)[0] + dnpod1 * (v)[1]) + \
                     dnlon1 * (dnpod2 * (v)[2] + dnpod1 * (v)[3]))
    val->p = BILINEAR(p);
    val->e = BILINEAR(e);
    val->Tm = BILINEAR(Tm);
    val->la = BILINEAR(la);
    val->ah = BILINEAR(ah);
    val->aw = BILINEAR(aw);
#undef BILINEAR
}

/* VMF3 empirical b and c coefficients (vmf3_ht.f90): spherical harmonics to degree 12. */
static void vmf3_bc(gtime_t time, const double *pos, double *bh, double *bw, double *ch,
                    double *cw)
{
    enum { NMAX = 12 };
    double V[NMAX + 1][NMAX + 1] = {{0}}, W[NMAX + 1][NMAX + 1] = {{0}};
    double harm[4], x, y, z, poldist, sum[4][5] = {{0}};
    int n, m, i, k;

    harmonics(time, harm);

    poldist = PI / 2.0 - pos[0];
    x = sin(poldist) * cos(pos[1]);
    y = sin(poldist) * sin(pos[1]);
    z = cos(poldist);

    /* Legendre polynomials */
    V[0][0] = 1.0;
    V[1][0] = z * V[0][0];
    for (n = 2; n <= NMAX; n++) {
        V[n][0] = ((2 * n - 1) * z * V[n - 1][0] - (n - 1) * V[n - 2][0]) / n;
    }
    for (m = 1; m <= NMAX; m++) {
        V[m][m] = (2 * m - 1) * (x * V[m - 1][m - 1] - y * W[m - 1][m - 1]);
        W[m][m] = (2 * m - 1) * (x * W[m - 1][m - 1] + y * V[m - 1][m - 1]);
        if (m < NMAX) {
            V[m + 1][m] = (2 * m + 1) * z * V[m][m];
            W[m + 1][m] = (2 * m + 1) * z * W[m][m];
        }
        for (n = m + 2; n <= NMAX; n++) {
            V[n][m] = ((2 * n - 1) * z * V[n - 1][m] - (n + m - 1) * V[n - 2][m]) / (n - m);
            W[n][m] = ((2 * n - 1) * z * W[n - 1][m] - (n + m - 1) * W[n - 2][m]) / (n - m);
        }
    }
    /* coefficients of mean, annual and semi-annual terms */
    i = 0;
    for (n = 0; n <= NMAX; n++) {
        for (m = 0; m <= n; m++, i++) {
            for (k = 0; k < 5; k++) {
                sum[0][k] += ANM_BH[i][k] * V[n][m] + BNM_BH[i][k] * W[n][m];
                sum[1][k] += ANM_BW[i][k] * V[n][m] + BNM_BW[i][k] * W[n][m];
                sum[2][k] += ANM_CH[i][k] * V[n][m] + BNM_CH[i][k] * W[n][m];
                sum[3][k] += ANM_CW[i][k] * V[n][m] + BNM_CW[i][k] * W[n][m];
            }
        }
    }
    *bh = seasonal(sum[0], harm);
    *bw = seasonal(sum[1], harm);
    *ch = seasonal(sum[2], harm);
    *cw = seasonal(sum[3], harm);
}

/* Fills the cache for this station and time. */
static void update_cache(gtime_t time, const double *pos)
{
    if (cache.valid && fabs(timediff(time, cache.time)) < 300.0 &&
        fabs(pos[0] - cache.pos[0]) < 1E-7 && fabs(pos[1] - cache.pos[1]) < 1E-7 &&
        fabs(pos[2] - cache.pos[2]) < 1.0) {
        return;
    }
    gpt3_values(time, pos, &cache.val);
    vmf3_bc(time, pos, &cache.bh, &cache.bw, &cache.ch, &cache.cw);
    cache.time = time;
    memcpy(cache.pos, pos, sizeof(cache.pos));
    cache.valid = 1;
}

/* Marini continued fraction normalized to 1 at zenith. */
static double marini(double sinel, double a, double b, double c)
{
    return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + a / (sinel + b / (sinel + c)));
}

extern int gpt3_mapf(gtime_t time, const double *pos, const double *azel, double *mfh,
                     double *mfw)
{
    /* height correction of the hydrostatic part [Niell, 1996] */
    const double a_ht = 2.53E-5, b_ht = 5.49E-3, c_ht = 1.14E-3;
    double sinel;

    if (!grid) return 0;

    if (azel[1] <= 0.0) {
        *mfh = 0.0;
        if (mfw) *mfw = 0.0;
        return 1;
    }
    update_cache(time, pos);

    sinel = sin(azel[1]);
    *mfh = marini(sinel, cache.val.ah, cache.bh, cache.ch) +
           (1.0 / sinel - marini(sinel, a_ht, b_ht, c_ht)) * pos[2] / 1000.0;
    if (mfw) *mfw = marini(sinel, cache.val.aw, cache.bw, cache.cw);
    return 1;
}

extern int gpt3_zenith(gtime_t time, const double *pos, double *zhd, double *zwd)
{
    /* Askne and Nordius refractivity constants (K/hPa, K^2/hPa) */
    const double k1 = 77.604, k2 = 64.79, k3 = 377600.0;
    const double gm = 9.80665, dMtr = 28.965E-3, Rg = 8.3143;
    double k2p, f;

    if (!grid) return 0;
    if (pos[2] < -1000.0 || pos[2] > 20000.0) return 0;

    update_cache(time, pos);

    /* Saastamoinen (saasthyd.f) with the GPT3 pressure */
    f = 1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00000028 * pos[2];
    *zhd = 0.0022768 * cache.val.p / f;

    /* asknewet.f */
    k2p = k2 - k1 * 18.0152 / 28.9644;
    *zwd = 1E-6 * (k2p + k3 / cache.val.Tm) * (Rg / dMtr) / (cache.val.la + 1.0) / gm *
           cache.val.e;
    return 1;
}

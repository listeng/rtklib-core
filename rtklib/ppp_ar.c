/*------------------------------------------------------------------------------
* ppp_ar.c : ppp ambiguity resolution
*
* Ported from MRTKLIB src/pos/mrtk_ppp_ar.c (ppp_amb_ILS, dual-frequency path),
* which comes from MADOCALIB / MALIB:
*
*          Copyright (C) 2026 H.SHIONO (MRTKLIB Project)
*          Copyright (C) 2023-2025 Cabinet Office, Japan
*          Copyright (C) 2024-2025 Lighthouse Technology & Consulting Co. Ltd.
*          Copyright (C) 2023-2025 Japan Aerospace Exploration Agency
*          Copyright (C) 2023-2025 TOSHIBA ELECTRONIC TECHNOLOGIES CORPORATION
*          Copyright (C) 2015- Mitsubishi Electric Corp.
*          Copyright (C) 2014 Geospatial Information Authority of Japan
*          Copyright (C) 2014 T.SUZUKI
*          Copyright (C) 2007-2023 T.TAKASU
*
*          SPDX-License-Identifier: BSD-2-Clause
*
* The method needs the uncombined PPP model (ionosphere estimated per
* satellite, IONOOPT_EST) with the satellite code and phase biases removed by
* absolute OSB from a Bias-SINEX file (see corr_meas() in ppp.c): the float
* phase-bias states are then integer ambiguities plus a receiver bias that is
* common to the satellites of one system and cancels between satellites.
*
*  1. single differences between satellites of one system, referenced to the
*     highest one
*  2. wide-lane (L1-L2) ambiguities of the SDs: rounded when close to an
*     integer, applied to the filter as tight pseudo-observations
*  3. L1 ambiguities of the SDs: integer least squares (LAMBDA) with a ratio
*     test; on failure the lowest rejected satellite is dropped and the search
*     repeated (partial ambiguity resolution)
*
* Adaptations to RTKLIB demo5: demo5 state layout (ppp.c), demo5 frequency
* index (code2idx) for the wavelengths, no triple-frequency EWL step, BDS-2
* and BDS-3 handled as one system (one receiver clock and bias in ppp.c).
*
* reference :
*    [1] CAO IS-QZSS-MDC-004, May, 2025
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MIN_AMB_RES     4       /* min number of ambiguities for ILS-AR */
#define CONST_AMB       0.01    /* constraint to fixed ambiguity (cycle) */
#define CONST_AMB_BDS   0.05    /* constraint to fixed BDS ambiguity (cycle) */
#define MAX_STD_FIX     0.15    /* max std-dev (3d-pos) to fix solution (m) */
#define MIN_MATCH_RATE  0.10    /* min match rate of 1st/2nd candidates to fix */
#define REL_MATCH_RATE  0.90    /* match rate to relax the ratio threshold */
#define MAX_FRAC_WL_FIX 0.20    /* max fraction to fix wide-lane (cycle) */
#define MAX_STD_WL_FIX  1.00    /* max std-dev to fix wide-lane (cycle) */
#define MAX_STD_NL_POS  1.50    /* max std-dev (3d-pos) to search L1 ambiguity (m) */
#define MAX_FIX_SHIFT   3.0     /* max fix-float position shift (float sigma) */

#define SQR(x)      ((x)*(x))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define MIN(x,y)    ((x)<(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)
#define SWAP_I(x,y) do {int    _tmp=x; x=y; y=_tmp;} while (0)
#define SWAP_D(x,y) do {double _tmp=x; x=y; y=_tmp;} while (0)

/* number and index of ekf states: must match ppp.c */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics?9:3)
#define NC(opt)     (NSYS)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
#define NI(opt)     ((opt)->ionoopt==IONOOPT_EST?MAXSAT:0)
#define ND(opt)     ((opt)->nf>=3?1:0)
#define NR(opt)     (NP(opt)+NC(opt)+NT(opt)+NI(opt)+ND(opt))
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)

/* wavelengths of the signals tracked in the first two slots (m), 0: none.
 * The slots hold the codes rinex.c assigned, so the frequency follows the
 * code (e.g. BDS B1I/B3I or B1I/B2b, Galileo E1/E5b) */
static void get_wavelength(const obsd_t *obs, const nav_t *nav, double *lam)
{
    double freq;
    int f;

    for (f=0;f<2;f++) {
        freq=obs->code[f]?sat2freq(obs->sat,obs->code[f],nav):0.0;
        lam[f]=freq>0.0?CLIGHT/freq:0.0;
    }
}
/* satellite usable for AR: dual-frequency phase with phase OSB of both signals
 * (without the satellite phase bias the ambiguity is not an integer) -------*/
static int usable_sat(rtk_t *rtk, const obsd_t *obs, const nav_t *nav, int exc,
                      double el, int sys)
{
    double elmask=MAX(rtk->opt.elmaskar,rtk->opt.elmin);
    int s=satsys(obs->sat,NULL);

    if (exc||s!=sys) return 0;
    if (s==SYS_GLO||s==SYS_SBS||s==SYS_IRN) return 0;
    if (s==SYS_CMP&&!rtk->opt.bdsmodear) return 0;
    if (!rtk->ssat[obs->sat-1].vsat[0]||el<elmask) return 0;
    if (obs->L[0]==0.0||obs->L[1]==0.0||!obs->code[0]||!obs->code[1]) return 0;
    if (!nav->osb_vpb[obs->sat-1][obs->code[0]-1]||
        !nav->osb_vpb[obs->sat-1][obs->code[1]-1]) return 0;
    return 1;
}
/* BDS GEO/IGSO: never a reference satellite, dropped first in partial AR ---*/
static int is_bds_geo_igso(int sat)
{
    int prn;
    if (satsys(sat,&prn)!=SYS_CMP) return 0;
    return prn<=10||prn==13||prn==16||(prn>=38&&prn<=40)||(prn>=56&&prn<=63);
}
/* generate satellite single differences -------------------------------------
* Groups of one system tracking the same pair of signals (e.g. GPS L1C+L2L and
* L1C+L2W are separate groups): only between those the receiver phase and code
* biases cancel. Each group is referenced to its highest satellite. */
static int gen_sat_sd(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav,
                      const int *exc, const double *azel, int *sat1, int *sat2,
                      int *iobs1, double *el1)
{
    const int sys[]={SYS_GPS,SYS_GAL,SYS_CMP,SYS_QZS,0};
    double el[MAXOBS],maxel;
    int i,j,k,m,ns=0,sat[MAXOBS],idx[MAXOBS],maxi,used[MAXOBS];

    for (i=0;sys[i];i++) {
        for (j=0;j<n&&j<MAXOBS;j++) {
            used[j]=!usable_sat(rtk,obs+j,nav,exc[j],azel[1+j*2],sys[i]);
        }
        for (;;) {
            /* reference: highest remaining satellite (no BDS GEO/IGSO) */
            for (j=0,maxi=-1,maxel=0.0;j<n&&j<MAXOBS;j++) {
                if (used[j]||is_bds_geo_igso(obs[j].sat)) continue;
                if (azel[1+j*2]>maxel) {
                    maxel=azel[1+j*2];
                    maxi=j;
                }
            }
            if (maxi<0) break;
            used[maxi]=1;

            /* same signals, sorted by elevation */
            for (j=m=0;j<n&&j<MAXOBS;j++) {
                if (used[j]) continue;
                if (obs[j].code[0]!=obs[maxi].code[0]||
                    obs[j].code[1]!=obs[maxi].code[1]) continue;
                used[j]=1;
                sat[m]=obs[j].sat;
                idx[m]=j;
                el[m++]=azel[1+j*2];
            }
            for (j=0;j<m-1;j++) for (k=j+1;k<m;k++) {
                if (el[j]>=el[k]) continue;
                SWAP_I(sat[j],sat[k]); SWAP_I(idx[j],idx[k]); SWAP_D(el[j],el[k]);
            }
            for (j=0;j<m;j++) {
                sat1[ns]=sat[j];
                sat2[ns]=obs[maxi].sat;
                iobs1[ns]=idx[j];
                el1[ns++]=el[j];
            }
        }
    }
    return ns;
}
/* search wide-lane integer ambiguity ---------------------------------------*/
static int search_amb_wl(rtk_t *rtk, const obsd_t *obs, const nav_t *nav,
                         const double *x, const double *P, int *sat1,
                         int *sat2, const int *iobs1, int ns, int *sats,
                         double *a, double *N, double *D)
{
    double lam[2],b,var,frac;
    int i,j1,j2,k1,k2,na=0,nx=rtk->nx;

    for (i=0;i<nx*ns;i++) D[i]=0.0;

    for (i=0;i<ns;i++) {
        if (sat1[i]<=0) continue;
        get_wavelength(obs+iobs1[i],nav,lam);

        j1=IB(sat1[i],0,&rtk->opt); j2=IB(sat1[i],1,&rtk->opt);
        k1=IB(sat2[i],0,&rtk->opt); k2=IB(sat2[i],1,&rtk->opt);

        if (lam[0]==0.0||lam[1]==0.0||x[j1]==0.0||x[j2]==0.0||
            x[k1]==0.0||x[k2]==0.0) {
            sat1[i]=sat2[i]=0;
            continue;
        }
        b=(x[j1]/lam[0]-x[j2]/lam[1])-(x[k1]/lam[0]-x[k2]/lam[1]);
        frac=fabs(ROUND(b)-b);
        var=P[j1+j1*nx]/SQR(lam[0])+P[j2+j2*nx]/SQR(lam[1])-
            2.0*P[j1+j2*nx]/(lam[0]*lam[1]);
        var+=P[k1+k1*nx]/SQR(lam[0])+P[k2+k2*nx]/SQR(lam[1])-
             2.0*P[k1+k2*nx]/(lam[0]*lam[1]);

        if (sqrt(var)>MAX_STD_WL_FIX||frac>MAX_FRAC_WL_FIX) {
            trace(3,"search_amb_wl: excluded sat=%3d-%3d a=%8.3f sig=%6.3f\n",
                  sat1[i],sat2[i],b,sqrt(var));
            sat1[i]=sat2[i]=0;
            continue;
        }
        sats[na]=sat1[i];
        a[na]=b;
        N[na]=ROUND(b);
        D[j1+na*nx]= 1.0/lam[0];
        D[j2+na*nx]=-1.0/lam[1];
        D[k1+na*nx]=-1.0/lam[0];
        D[k2+na*nx]= 1.0/lam[1];
        rtk->ssat[sat1[i]-1].fix[1]=rtk->ssat[sat2[i]-1].fix[1]=1;
        na++;
    }
    trace(3,"search_amb_wl: fixed wide-lane %d / %d\n",na,ns);
    return na;
}
/* SD L1 ambiguities and their covariance -----------------------------------*/
static int gen_sd_n1(rtk_t *rtk, const obsd_t *obs, const nav_t *nav,
                     const double *x, const double *P, const int *sat1,
                     const int *sat2, const int *iobs1, int ns, int *sats,
                     int *sats2, double *a, double *Q, double *D)
{
    double lam[2],q;
    int i,j,k,na=0,ind[MAXOBS][2],nx=rtk->nx;

    for (i=0;i<nx*ns;i++) D[i]=0.0;

    for (i=0;i<ns;i++) {
        if (sat1[i]<=0) continue;
        get_wavelength(obs+iobs1[i],nav,lam);
        j=IB(sat1[i],0,&rtk->opt);
        k=IB(sat2[i],0,&rtk->opt);
        if (lam[0]==0.0||x[j]==0.0||x[k]==0.0) continue;

        sats[na]=sat1[i];
        sats2[na]=sat2[i];
        D[j+na*nx]= 1.0/lam[0];
        D[k+na*nx]=-1.0/lam[0];
        ind[na][0]=j;
        ind[na][1]=k;
        na++;
    }
    /* a=D'*x, Q=D'*P*D (two non-zeros per column) */
    for (j=0;j<na;j++) {
        int j0=ind[j][0],j1=ind[j][1];
        a[j]=D[j0+j*nx]*x[j0]+D[j1+j*nx]*x[j1];
        for (i=j;i<na;i++) {
            int i0=ind[i][0],i1=ind[i][1];
            q =D[i0+i*nx]*P[i0+j0*nx]*D[j0+j*nx];
            q+=D[i0+i*nx]*P[i0+j1*nx]*D[j1+j*nx];
            q+=D[i1+i*nx]*P[i1+j0*nx]*D[j0+j*nx];
            q+=D[i1+i*nx]*P[i1+j1*nx]*D[j1+j*nx];
            Q[i+j*na]=Q[j+i*na]=q;
        }
    }
    return na;
}
/* integer least squares with ratio test ------------------------------------*/
static int search_amb_lambda(rtk_t *rtk, const int *sats, const int *sats2,
                             const double *Q, const double *a, double *N,
                             int na, int *exc_flag)
{
    const double thres_fact[]={5.0,5.0,3.0,2.0,1.5};
    double s[2],thres;
    int i,nm=0;

    if (lambda(na,2,a,Q,N,s)||s[0]<=0.0) {
        trace(2,"search_amb_lambda: lambda error\n");
        return 0;
    }
    /* stricter ratio for few ambiguities */
    thres=rtk->opt.thresar[0];
    if (na-MIN_AMB_RES<(int)(sizeof(thres_fact)/sizeof(thres_fact[0]))) {
        thres*=thres_fact[na-MIN_AMB_RES];
    }
    /* candidates that differ between the best and second best solutions */
    for (i=0;i<MAXSAT;i++) exc_flag[i]=0;
    for (i=0;i<na;i++) {
        if (fabs(N[i]-N[i+na])<0.001) nm++;
        else exc_flag[sats[i]-1]=1;
    }
    if ((double)nm/na>REL_MATCH_RATE) thres*=0.8;
    if ((double)nm/na<MIN_MATCH_RATE) thres=99.99;

    rtk->sol.ratio=(float)MIN(s[1]/s[0],999.9);
    if (s[1]/s[0]<thres) {
        trace(2,"search_amb_lambda: failed na=%d ratio=%.2f/%.2f\n",na,
              s[1]/s[0],thres);
        return 0;
    }
    for (i=0;i<na;i++) {
        rtk->ssat[sats[i]-1].fix[0]=rtk->ssat[sats2[i]-1].fix[0]=2;
    }
    trace(2,"search_amb_lambda: fixed na=%d ratio=%.2f/%.2f\n",na,s[1]/s[0],
          thres);
    return na;
}
/* constrain states to the integer ambiguities ------------------------------*/
static int update_states(double *x, double *P, const double *D, int nx,
                         const double *N, const double *a, int na,
                         const int *sats)
{
    double *R,*b;
    int i,info;

    if (na<=0) return 0;

    R=zeros(na,na); b=mat(na,1);
    for (i=0;i<na;i++) {
        b[i]=N[i]-a[i];
        R[i+i*na]=SQR(satsys(sats[i],NULL)==SYS_CMP?CONST_AMB_BDS:CONST_AMB);
    }
    if ((info=filter(x,P,D,b,R,nx,na))) {
        trace(1,"update_states: filter error info=%d\n",info);
    }
    free(R); free(b);
    return info;
}
/* drop the lowest of the rejected satellites (partial AR) ------------------*/
static void exc_sat_par(int *sat1, int *sat2, const double *el1, int ns,
                        const int *exc_flag)
{
    double el,minel=9999.9;
    int i,mini=-1;

    for (i=0;i<ns;i++) {
        if (!sat1[i]||!exc_flag[sat1[i]-1]) continue;
        el=el1[i];
        /* BDS GEO/IGSO and QZSS first */
        if (is_bds_geo_igso(sat1[i])||satsys(sat1[i],NULL)==SYS_QZS) {
            el-=PI/2.0;
        }
        if (el<minel) {
            minel=el;
            mini=i;
        }
    }
    if (mini<0) { /* no rejected candidate: drop the lowest */
        for (i=0;i<ns;i++) {
            if (sat1[i]&&el1[i]<minel) {
                minel=el1[i];
                mini=i;
            }
        }
    }
    if (mini<0) return;
    trace(2,"exc_sat_par: excluded sat=%3d-%3d el=%4.1f\n",sat1[mini],
          sat2[mini],el1[mini]*R2D);
    sat1[mini]=sat2[mini]=0;
}
/* ambiguity resolution by ILS ----------------------------------------------*/
static int ppp_amb_ILS(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                       const nav_t *nav, const double *azel, double *x,
                       double *P)
{
    double *D,*Q,*xp,*Pp,a[MAXOBS],N[MAXOBS*2],el1[MAXOBS],std[3];
    int sat1[MAXOBS],sat2[MAXOBS],iobs1[MAXOBS];
    int sats[MAXOBS],sats2[MAXOBS],exc_flag[MAXSAT];
    int i,ns,na,info,nx=rtk->nx;

    ns=gen_sat_sd(rtk,obs,n,nav,exc,azel,sat1,sat2,iobs1,el1);
    if (ns<MIN_AMB_RES) {
        trace(3,"ppp_amb_ILS: too few satellite SDs ns=%d\n",ns);
        return 0;
    }
    D=zeros(nx,ns); Q=mat(ns,ns); xp=mat(nx,1); Pp=mat(nx,nx);

    /* wide-lane */
    na=search_amb_wl(rtk,obs,nav,x,P,sat1,sat2,iobs1,ns,sats,a,N,D);
    if (na==0||update_states(x,P,D,nx,N,a,na,sats)) {
        free(D); free(Q); free(xp); free(Pp);
        return 0;
    }
    matcpy(xp,x,nx,1);
    matcpy(Pp,P,nx,nx);

    /* L1 (narrow-lane) */
    for (i=0;i<3;i++) std[i]=sqrt(P[i+i*nx]);
    na=0;
    if (norm(std,3)<MAX_STD_NL_POS) {
        for (i=0;i<MAX(rtk->opt.armaxiter,1);i++) {
            na=gen_sd_n1(rtk,obs,nav,x,P,sat1,sat2,iobs1,ns,sats,sats2,a,Q,D);
            if (na<MIN_AMB_RES) {
                na=0;
                break;
            }
            if ((na=search_amb_lambda(rtk,sats,sats2,Q,a,N,na,exc_flag))>0) {
                break;
            }
            exc_sat_par(sat1,sat2,el1,ns,exc_flag);
        }
    }
    else {
        trace(3,"ppp_amb_ILS: position std too large %.3f\n",norm(std,3));
    }
    if (na==0) {
        free(D); free(Q); free(xp); free(Pp);
        return 0;
    }
    info=update_states(x,P,D,nx,N,a,na,sats);

    /* position std test, and consistency of the fixed with the float position:
     * a correct fix moves the position within the float uncertainty; a wrong
     * set of integers (accepted by the ratio test on a short or noisy arc)
     * pulls it decimetres away */
    for (i=0;i<3;i++) std[i]=sqrt(P[i+i*nx]);
    if (!info) {
        double sh=0.0,sf;
        for (i=0;i<3;i++) {
            sf=sqrt(Pp[i+i*nx]);
            if (sf>0.0) sh=MAX(sh,fabs(x[i]-xp[i])/sf);
        }
        if (sh>MAX_FIX_SHIFT) {
            trace(2,"ppp_amb_ILS: fix rejected, position shift %.1f sigma\n",sh);
            info=-1;
        }
    }
    if (info||norm(std,3)>MAX_STD_FIX) {
        for (i=0;i<na;i++) {
            rtk->ssat[sats[i]-1].fix[0]=rtk->ssat[sats2[i]-1].fix[0]=0;
        }
        matcpy(x,xp,nx,1);
        matcpy(P,Pp,nx,nx);
        free(D); free(Q); free(xp); free(Pp);
        return 0;
    }
    trace(2,"ppp_amb_ILS: fixed nfix=%d nsd=%d std=%.3f\n",na,ns,norm(std,3));
    free(D); free(Q); free(xp); free(Pp);
    return 1;
}
/* ambiguity resolution in ppp ----------------------------------------------
* args   : rtk_t  *rtk      IO  rtk control/result struct
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          int    *exc      I   excluded satellite flags
*          nav_t  *nav      I   navigation data
*          double *azel     I   azimuth/elevation angles (rad)
*          double *x        IO  float states, fixed on return 1
*          double *P        IO  covariance of the states
* return : 1: fixed, 0: not fixed
*-----------------------------------------------------------------------------*/
extern int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                  const nav_t *nav, const double *azel, double *x, double *P)
{
    if (n<=0||rtk->opt.modear!=ARMODE_PPPAR) return 0;

    /* needs the uncombined model and dual-frequency phase */
    if (rtk->opt.ionoopt!=IONOOPT_EST||rtk->opt.nf<2) return 0;

    return ppp_amb_ILS(rtk,obs,n,exc,nav,azel,x,P);
}

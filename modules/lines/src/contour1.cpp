/*
This file was taken from Calife library and slightly modified:
In IMCT change from LISTE to std::vector    
    old: LISTE *liste;
    new: std::vector<CHAINE*> liste;
*/


/*
			contour.c
			---------
			Radu Horaud
			
			version calife
			Matthieu Herrb - Fevrier 90

 Extraction de chaines de contour a partir de l'image de gradient
 Version sur ima1
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//#include "liste.h"
//#include "sgdef.h"
#include "lines/ctdef.hpp"

#define  FORWARD  1
#define  BACKWARD -1
#define  HALFPOINTS (MAXPOINTS / 2)
#define	 NCAND	5
#define  ERASEVALUE 1

namespace jafar{
  namespace lines{
    static unsigned char *gr;
    static int maxi, lines, columns, pcontour;
    static int *iforw, *jforw, *iback, *jback, nforw, nback;
    
    static int suci_1[NCAND] = {0, 1, -1, 1, -1},
        sucj_1[NCAND] = {1, 1, 1, 0, 0},
    
        suci_2[NCAND] = {-1, 0, -1, 1, -1},
        sucj_2[NCAND] = {1, 1, 0, 1, -1},
    
        suci_3[NCAND] = {-1, -1, -1, 0, 0},
        sucj_3[NCAND] = {0, 1, -1, 1, -1},
    
        suci_4[NCAND] = {-1, -1, 0, -1, 1},
        sucj_4[NCAND] = {-1, 0, -1, 1, -1},
    
        suci_5[NCAND] = {0, -1, 1, -1, 1},
        sucj_5[NCAND] = {-1, -1, -1, 0, 0},
    
        suci_6[NCAND] = {1, 0, 1, -1, 1},
        sucj_6[NCAND] = {-1, -1, 0, -1, 1},
    
        suci_7[NCAND] = {1, 1, 1, 0, 0},
        sucj_7[NCAND] = {0, -1, 1, -1, 1},
    
        suci_8[NCAND] = {1, 1, 0, 1, -1},
        sucj_8[NCAND] = {1, 0, 1, -1, 1};
    
    /*
    static void FDEC(seuilhy, (IMCT *output, int c_i0, int c_j0, int di0, int dj0,
            int sh, int sb, int mlen));
    static int FDEC(confcont, (int ip,int seuil, int il, int ic,
            int sb, int mlen, int *sens, int *direction));
    static int FDEC(get_direction, (int i, int j, int in, int jn));
    static void FDEC(destroy_leftright, (int dir, int ip));
    */
    
    void ExtractContours1(char *input, IMCT *output, int di,int dj,int c_i0,int c_j0,int di0,int dj0,
                              int up_thr,int down_thr, int min_length)
            /*
        char *input;
        IMCT *output;
        int di, dj;
        int c_i0, c_j0, di0, dj0;
        int up_thr, down_thr;
        int  min_length;
            */
    {
        /* copy the pointers */
        lines = di;
        columns = dj;
        maxi = lines * columns;
        pcontour = 0;
    
        /* make some working buffers */
        if ((iforw = (int *) malloc(HALFPOINTS * sizeof(int))) == FALSE) {
      printf("Unable to make working buffers\n");
      return;
        }
        if ((jforw = (int *) malloc(HALFPOINTS * sizeof(int))) == FALSE) {
      printf("Unable to make working buffers\n");
      free((char *)iforw);
      return;
        }
        if ((iback = (int *) malloc(HALFPOINTS * sizeof(int))) == FALSE) {
      printf("Unable to make working buffers\n");
      free((char *)iforw);
      free((char *)jforw);
      return;
        }
        if ((jback = (int *) malloc(HALFPOINTS * sizeof(int))) == FALSE) {
      printf("Unable to make working buffers\n");
      free((char *)iforw);
      free((char *)jforw);
      free((char *)iback);
      return;
        }
        gr = (unsigned char *)input;
        seuilhy(output, c_i0, c_j0, di0, dj0, up_thr, down_thr, min_length);
    #ifndef VXWORKS
        /*printf(" -- %d chaines\n", nimct(output));*/
    #endif
    
        free((char *)iforw);
        free((char *)jforw);
        free((char *)iback);
        free((char *)jback);
        return;
    }
    
    
    
    void seuilhy(IMCT *output,int c_i0,int c_j0, int di0,int dj0,int sh,int sb, int mlen)
        /* 
        IMCT *output;
        int c_i0, c_j0, di0, dj0;
        int sh, sb;
        int  mlen;*/
    {
        int  il, ic, ip, in, i, j;
        int x0, y0, x1, y1;
        int seuil;
        int  sens = 0;
        int  direction = 0;
        int  npp;
        //CHAINE *newchain; //, *ct_cree();
    
        /* init the chains */
        dict(output) = lines;
        djct(output) = columns;
        nimct(output) = 0;
    
        for (il = c_i0; il < c_i0 + di0; il++) {
      in = il*columns;
      for (ic = c_j0; ic < c_j0 + dj0; ic++) {
          ip = in + ic;	/* (il,ic) is the position of current pixel;
            * ip idem. */
          seuil = sh;
          sens = 0;
          direction = 0;
          nforw = nback = 0;
          if (gr[ip] >= seuil) {
        if (confcont(ip, seuil, il, ic, sb, mlen,
              &sens, &direction) == TRUE) {
            npp = nforw + nback;
            //newchain = ct_cree(npp);
            
            CHAINE *newchain = new CHAINE; // *ct_cree;
            initChain(newchain, npp);
            
            ctnum(newchain) = nimct(output);
            j = 0;
            for (i = nback - 1; i >= 0; --i) {	/* reverse and store
                  * back buffer */
          ct_y(newchain, j) = jback[i];
          ct_x(newchain, j) = iback[i];
          pcontour++;
          ++j;
            }
            for (i = 0; i <= nforw - 1; ++i) {	/* store forward buffer */
          ct_y(newchain, j) = jforw[i];
          ct_x(newchain, j) = iforw[i];
          pcontour++;
          ++j;
            }
            /* Teste si la chaine est fermee */
            if (nback != 0) {
          y0 = jback[nback-1];
          x0 = iback[nback-1];
            } else {
          y0 = jforw[0];
          x0 = iforw[0];
            }
            if (nforw != 0) {
          y1 = jforw[nforw-1];
          x1 = iforw[nforw-1];
            } else {
          y1 = jback[0];
          x1 = iback[0];
            }
            if (abs(x1-x0) <= 1 && abs(y1-y0) <= 1) {
          /* chaine fermee */
          ctflag(newchain) = CT_FERMEE;
            }
            //listect(output) = lcons(&newchain, typchaine,listect(output));
            listect(output).push_back(newchain);
            nimct(output)++;
        } /* if (confcont()) */
          }
      } /* for ic */
        } /* for il */
        return;
    }
    
    
    /*
    * Suivi d'une chaine avec seuillage par hystersis
    */
    
    int confcont(int ip,int seuil,int il,int ic,int sb,int mlen,int* sens,int* direction)
        /*int  ip;
        int seuil, sb;
        int  mlen;
        int  il, ic;
        int *sens;
        int *direction;*/
    {
        register int i;
        register int im;
        register int ik;
        int  in;
        int  ik1, im1;
        int  coups;
        int  ldirection;
        if (nforw >= HALFPOINTS && *sens == FORWARD) /* try to go forward */
      return (FALSE);
        if (ic >= 0 && ic < columns && il >= 0 && il <lines ) {
      /* ip should be in the image */
      if (gr[ip] >= seuil && nback < HALFPOINTS) {
        gr[ip] = ERASEVALUE;
          seuil = sb;
          switch (*sens) {
            case 0:		/* start a new chain and go forward first */
        nforw = nback = 0;
        iforw[nforw] = il;
        jforw[nforw] = ic;
        nforw++;
        *sens = FORWARD;
        break;
            case FORWARD:
        iforw[nforw] = il;	/* continue forward */
        jforw[nforw] = ic;
        nforw++;
        break;
            case BACKWARD:
        iback[nback] = il;	/* continue backward */
        jback[nback] = ic;
        nback++;
        break;
          }
          switch (*direction) {
            case 0:		/* starting point */
        coups = 0;
        for (ik = 0; ik <= 1; ik++) {	/* skip the first line of
                * window */
            ik1 = il + ik;
            for (im = (-1); im <= 1; im++) {
          im1 = ic + im;
          in = ik1 * columns + im1;
          if ((in == ip) || (in == (ip - 1)))
              continue;	/* skip those! */
          ldirection = *direction = get_direction(il, ic,
                    ik1, im1);
          if (confcont(in, seuil, ik1, im1, sb,
                mlen, sens, direction) == TRUE)
              if (++coups >= 2)
            goto sortie;
    
            }
        }
      sortie:
        if ((nback + nforw) > mlen)
            return (TRUE);
        else
            return (FALSE);
            case 1:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_1[i];
            im1 = ic + sucj_1[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 2:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_2[i];
            im1 = ic + sucj_2[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 3:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_3[i];
            im1 = ic + sucj_3[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 4:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_4[i];
            im1 = ic + sucj_4[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 5:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_5[i];
            im1 = ic + sucj_5[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 6:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_6[i];
            im1 = ic + sucj_6[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 7:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_7[i];
            im1 = ic + sucj_7[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
        break;
            case 8:
        for (i = 0; i < NCAND; ++i) {
            ik1 = il + suci_8[i];
            im1 = ic + sucj_8[i];
            in = ik1 * columns + im1;
            ldirection = *direction = get_direction(il, ic, ik1, im1);
            if (confcont(in, seuil, ik1, im1, sb,
            mlen, sens, direction) == TRUE) {
          destroy_leftright(ldirection, ip);
          return (TRUE);
            }
        }
          }
          *sens = BACKWARD;	/* change sens and try to go bakcward */
          return (TRUE);
      }
      return (FALSE);
        }
        return (FALSE);
    }
    
    
    
    int get_direction(int i,int j,int in,int jn)
        /*int  i, j, in, jn;*/
    {
        static int tabdir[3][3] = {{4, 3, 2}, {5, 0, 1}, {6, 7, 8}};
    
        int  ii, jj;
        ii = in - i;
        jj = jn - j;
    #if (1)
        return(tabdir[ii + 1][jj + 1]);
    #else
        if (ii == 0) {
      if (jj == 1)
          return (1);
      if (jj == -1)
          return (5);
        }
        if (ii == 1) {
      if (jj == 1)
          return (8);
      if (jj == 0)
          return (7);
      if (jj == -1)
          return (6);
        }
        if (ii == -1) {
      if (jj == 1)
          return (2);
      if (jj == 0)
          return (3);
      if (jj == -1)
          return (4);
        }
        printf("getdirection: configuration impossible: %d %d %d %d\n",
        i, j, in, jn);
        return(0);
    #endif    
    }
    
    
    
    void destroy_leftright(int dir,int ip)
        /*int  dir, ip;*/
    {
        register int il, ir;
    
        switch (dir) {
          case 1:
      il = ip - columns + 1;
      ir = ip + columns + 1;
      break;
          case 2:
      il = ip - columns;
      ir = ip + 1;
      break;
          case 3:
      il = ip - columns + 1;
      ir = ip - columns - 1;
      break;
          case 4:
      il = ip - columns;
      ir = ip - 1;
      break;
          case 5:
      il = ip - columns - 1;
      ir = ip + columns - 1;
      break;
          case 6:
      il = ip - 1;
      ir = ip + columns;
      break;
          case 7:
      il = ip + columns - 1;
      ir = ip + columns + 1;
      break;
          case 8:
      il = ip + 1;
      ir = ip + columns;
      break;
          default:
      printf("destroy_leftright(): no such direction %d\n", dir);
      return;
        }
    
        if (il >= 0 && il < maxi)
          gr[il] = ERASEVALUE;
        if (ir >= 0 && ir < maxi)
          gr[ir] = ERASEVALUE;
    
        return;
    }
    
    /** p function
    initialize chaine
    */
    void initChain(CHAINE* chain, int nPoints){
      chain->x = new short[nPoints];
      chain->y = new short[nPoints];
      
      chain->npoints = nPoints;
      chain->flag= 0;
      chain->local = NULL;
              
      
    }
  } // namespace lines
} // namespace jafar



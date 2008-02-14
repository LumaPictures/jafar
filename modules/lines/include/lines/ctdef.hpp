/*
This file was taken from Calife library and slightly modified:
In IMCT change from LISTE to std::vector    
    old: LISTE *liste;
    new: std::vector<CHAINE*> liste;
The structs are converted to classes.
*/

/*----------------------------------------------------------------------
     
                         -- C N R S -- 
         Laboratoire d'Automatique et d'Analyse des Systemes 
                 7 Avenue du colonel Roche   
                     31 077 Toulouse Cedex  

     
  Fichier              : /usr/local/calife/common/ctdef.h
  Fonction             : 
     
  Date de creation     : Jeudi 9  Aout 14:07:04 1990
  Date de modification : Vendredi 18 Septembre 11:56:59 1992
  Nb de lignes         : 117 
     
  Auteur               : (Calife)
  Groupe               : Robotique et Intelligence Artificielle  
     
  Copyright (C) 1990  LAAS-CNRS. 
----------------------------------------------------------------------*/
/**
 ** Definition du type chaine de points de contour
 **
 ** Matthieu Herrb - Avril 90
 ** Michel Devy    - Aout  90 - ajout d'attribut sur chaque contours
 **/

 
#ifndef DEFCT
#define DEFCT

//#include "erreur_calife.h"
//#include "liste.h"
#include <vector>


namespace jafar{
  namespace lines{
    #define MAXPOINTS    2000     /* max number of points for a chain */
    
    #ifndef TRUE
    #define TRUE 1
    #endif
    
    #ifndef FALSE
    #define FALSE 0
    #endif
    
    /* le type chaine de points de contours */
    
    class CHAINE /* an edge chain */
    {
    public:
        int num;		/* numero de la chaine */
        int flag;		/* flags associe's a` la chaine */
        short npoints;	/* nombre de points */
        char *local;        /* attributs locaux */
        short *x, *y;	/* tableaux des coordonnees */
    };
    
    
    /* macros d'acces aux champs d'une chaine */
    
    #define ctnum(c)  ((c)->num)
    #define ctflag(c) ((c)->flag)
    #define ctnpts(c) ((c)->npoints)
    #define ctlocal(c) ((c)->local)
    #define ctx(c)    ((c)->x)
    #define cty(c)    ((c)->y)
    #define ct_x(c,i) ((c)->x[i])
    #define ct_y(c,i) ((c)->y[i])
    
    /* Le flag pour chaine fermee */
    
    #define CT_FERMEE 0x01
    
    /* le type des chaines pour utilisation en listes */
    
    #define typchaine 120
    
    /* Codes d'erreur */
    #define Cf_Ctfic 155
    
    #define ERR_CT_INIT   ErrNum(Cf_Ctfic, 1)
    
    /*------------------------------------------------------------------------*/
    
    /**
    **  Le type image de chaines de points de contour
    **/
    
    class IMCT{
    public:
        //LISTE *liste;		/* liste des chaines */
        std::vector<CHAINE*> liste;
        int nchains;		/* nombre de chaines */
        int di, dj;			/* taille de l'image originale */
    };
    
    //extern int nchain;
    
    /* macros pour acces aux champs de ces images */
    
    #define listect(c) ((c)->liste)
    #define dict(c)    ((c)->di)
    #define djct(c)    ((c)->dj)
    #define nimct(c)   ((c)->nchains)
    
    
    /*------------------------------------------------------------------------*/
    
    /**
    ** prototypes des fonctions du module
    **/
    //#include "contours_proto.h"
      
      void ExtractContours1(char *input, IMCT* output, int di,int dj, int c_i0, int c_j0,int di0,int dj0, int up_thr, int down_thr, int  min_length);
      void seuilhy(IMCT* output, int c_i0,int c_j0, int di0, int dj0, int sh, int sb, int  mlen);
      int confcont(int  ip, int seuil, int il, int ic, int sb, int  mlen, int *sens, int *direction);
      int get_direction(int  i,int j, int in, int jn);
      void destroy_leftright(int  dir, int ip);
      void initChain(CHAINE* chain, int nPoints);
    
    
  } // namespace lines
} // namspace jafar
#endif

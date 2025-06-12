#include "42.h"
#define EXTERN extern
#include "Ac.h"
#undef EXTERN

long ScTlmEnabled, MaxScCtr;
long ScBTlmEnabled, MaxScBCtr;
long ScGTlmEnabled, MaxScGCtr;
long ScWhlTlmEnabled, MaxScWhlCtr;
long ScGyroTlmEnabled, MaxScGyroCtr;
long ScMAGTlmEnabled, MaxScMAGCtr;
long ScCSSTlmEnabled, MaxScCSSCtr;
long ScFSSTlmEnabled, MaxScFSSCtr;
long ScSTTlmEnabled, MaxScSTCtr;
long ScGPSTlmEnabled, MaxScGPSCtr;
long ScAccelTlmEnabled, MaxScAccelCtr;
long ScShakerTlmEnabled, MaxScShakerCtr;
/******************************************************************************/
void WriteScToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"Sc.csv");
               else if (Nsc<=10) sprintf(FileName,"Sc%1ld.csv",Isc);
               else sprintf(FileName,"Sc%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"Sc_Time");
               for(i=0;i<4;i++) {
                  fprintf(outfile[Isc],",Sc_qn_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_wn_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_PosR_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_VelR_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_svb_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_bvb_%ld",i+1);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",Sc_Hvb_%ld",i+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(i=0;i<4;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->qn[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->wn[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->PosR[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->VelR[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->svb[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->bvb[i]);
               }
               for(i=0;i<3;i++) {
                  fprintf(outfile[Isc],",%18.12le",S->Hvb[i]);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScBToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScB.csv");
               else if (Nsc<=10) sprintf(FileName,"ScB%1ld.csv",Isc);
               else sprintf(FileName,"ScB%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScB_Time");
               for(k=0;k<S->Nb;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_B%ld_wn_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Nb;k++) {
                  for(i=0;i<4;i++) {
                     fprintf(outfile[Isc],",Sc_B%ld_qn_%ld",k+1,i+1);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScBCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nb;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->B[k].wn[i]);
                  }
               }
               for(k=0;k<S->Nb;k++) {
                  for(i=0;i<4;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->B[k].qn[i]);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScGToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScG.csv");
               else if (Nsc<=10) sprintf(FileName,"ScG%1ld.csv",Isc);
               else sprintf(FileName,"ScG%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScG_Time");
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_G%ld_Pos_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_G%ld_PosRate_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_G%ld_Ang_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_G%ld_AngRate_%ld",k+1,i+1);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScGCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->G[k].Pos[i]);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->G[k].PosRate[i]);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->G[k].Ang[i]);
                  }
               }
               for(k=0;k<S->Ng;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->G[k].AngRate[i]);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScWhlToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScWhl.csv");
               else if (Nsc<=10) sprintf(FileName,"ScWhl%1ld.csv",Isc);
               else sprintf(FileName,"ScWhl%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScWhl_Time");
               for(k=0;k<S->Nw;k++) {
                  fprintf(outfile[Isc],",Sc_Whl%ld_H",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScWhlCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nw;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->Whl[k].H);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScGyroToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScGyro.csv");
               else if (Nsc<=10) sprintf(FileName,"ScGyro%1ld.csv",Isc);
               else sprintf(FileName,"ScGyro%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScGyro_Time");
               for(k=0;k<S->Ngyro;k++) {
                  fprintf(outfile[Isc],",Sc_Gyro%ld_TrueRate",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScGyroCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Ngyro;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->Gyro[k].TrueRate);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScMAGToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScMAG.csv");
               else if (Nsc<=10) sprintf(FileName,"ScMAG%1ld.csv",Isc);
               else sprintf(FileName,"ScMAG%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScMAG_Time");
               for(k=0;k<S->Nmag;k++) {
                  fprintf(outfile[Isc],",Sc_MAG%ld_Field",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScMAGCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nmag;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->MAG[k].Field);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScCSSToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScCSS.csv");
               else if (Nsc<=10) sprintf(FileName,"ScCSS%1ld.csv",Isc);
               else sprintf(FileName,"ScCSS%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScCSS_Time");
               for(k=0;k<S->Ncss;k++) {
                  fprintf(outfile[Isc],",Sc_CSS%ld_Valid",k+1);
               }
               for(k=0;k<S->Ncss;k++) {
                  fprintf(outfile[Isc],",Sc_CSS%ld_Illum",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScCSSCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Ncss;k++) {
                  fprintf(outfile[Isc],",%ld",S->CSS[k].Valid);
               }
               for(k=0;k<S->Ncss;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->CSS[k].Illum);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScFSSToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScFSS.csv");
               else if (Nsc<=10) sprintf(FileName,"ScFSS%1ld.csv",Isc);
               else sprintf(FileName,"ScFSS%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScFSS_Time");
               for(k=0;k<S->Nfss;k++) {
                  fprintf(outfile[Isc],",Sc_FSS%ld_Valid",k+1);
               }
               for(k=0;k<S->Nfss;k++) {
                  for(i=0;i<2;i++) {
                     fprintf(outfile[Isc],",Sc_FSS%ld_SunAng_%ld",k+1,i+1);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScFSSCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nfss;k++) {
                  fprintf(outfile[Isc],",%ld",S->FSS[k].Valid);
               }
               for(k=0;k<S->Nfss;k++) {
                  for(i=0;i<2;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->FSS[k].SunAng[i]);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScSTToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScST.csv");
               else if (Nsc<=10) sprintf(FileName,"ScST%1ld.csv",Isc);
               else sprintf(FileName,"ScST%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScST_Time");
               for(k=0;k<S->Nst;k++) {
                  fprintf(outfile[Isc],",Sc_ST%ld_Valid",k+1);
               }
               for(k=0;k<S->Nst;k++) {
                  for(i=0;i<4;i++) {
                     fprintf(outfile[Isc],",Sc_ST%ld_qn_%ld",k+1,i+1);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScSTCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nst;k++) {
                  fprintf(outfile[Isc],",%ld",S->ST[k].Valid);
               }
               for(k=0;k<S->Nst;k++) {
                  for(i=0;i<4;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->ST[k].qn[i]);
                  }
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScGPSToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      long i;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScGPS.csv");
               else if (Nsc<=10) sprintf(FileName,"ScGPS%1ld.csv",Isc);
               else sprintf(FileName,"ScGPS%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScGPS_Time");
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Valid",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Rollover",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Week",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Sec",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_GPS%ld_PosN_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_GPS%ld_VelN_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_GPS%ld_PosW_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",Sc_GPS%ld_VelW_%ld",k+1,i+1);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Lng",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Lat",k+1);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",Sc_GPS%ld_Alt",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScGPSCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%ld",S->GPS[k].Valid);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%ld",S->GPS[k].Rollover);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%ld",S->GPS[k].Week);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->GPS[k].Sec);
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->GPS[k].PosN[i]);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->GPS[k].VelN[i]);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->GPS[k].PosW[i]);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  for(i=0;i<3;i++) {
                     fprintf(outfile[Isc],",%18.12le",S->GPS[k].VelW[i]);
                  }
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->GPS[k].Lng);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->GPS[k].Lat);
               }
               for(k=0;k<S->Ngps;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->GPS[k].Alt);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScAccelToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScAccel.csv");
               else if (Nsc<=10) sprintf(FileName,"ScAccel%1ld.csv",Isc);
               else sprintf(FileName,"ScAccel%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScAccel_Time");
               for(k=0;k<S->Nacc;k++) {
                  fprintf(outfile[Isc],",Sc_Accel%ld_TrueAcc",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScAccelCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nacc;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->Accel[k].TrueAcc);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScShakerToCsv(void)
{
      static FILE **outfile;
      struct SCType *S;
      char FileName[80];
      long Isc;
      long k;
      static long First = 1;
      static long OutCtr = 1000000000;

      if (First) {
         First = 0;
         outfile = (FILE**) calloc(Nsc,sizeof(FILE *));
         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               if (Nsc==1) sprintf(FileName,"ScShaker.csv");
               else if (Nsc<=10) sprintf(FileName,"ScShaker%1ld.csv",Isc);
               else sprintf(FileName,"ScShaker%02ld.csv",Isc);
               outfile[Isc] = FileOpen(InOutPath,FileName,"w");

               fprintf(outfile[Isc],"ScShaker_Time");
               for(k=0;k<S->Nsh;k++) {
                  fprintf(outfile[Isc],",Sc_Shaker%ld_Output",k+1);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

      OutCtr++;
      if (OutCtr > MaxScShakerCtr) {
         OutCtr = 1;
         for (Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               S = &SC[Isc];
               fprintf(outfile[Isc],"%18.12le",SimTime);
               for(k=0;k<S->Nsh;k++) {
                  fprintf(outfile[Isc],",%18.12le",S->Shaker[k].Output);
               }
               fprintf(outfile[Isc],"\n");
            }
         }
      }

}
/******************************************************************************/
void WriteScVarsToCsv(void)
{
      FILE *infile;
      char response[80],junk[80],newline;
      double dt;
      static long First = 1;

      if (First) {
         First = 0;

         infile = FileOpen(InOutPath,"Inp_ScOutput.txt","r");
         fscanf(infile,"%[^\n] %[\n]",junk,&newline);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("Sc timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScBTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScB timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScBCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScGTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScG timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScGCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScWhlTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScWhl timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScWhlCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScGyroTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScGyro timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScGyroCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScMAGTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScMAG timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScMAGCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScCSSTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScCSS timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScCSSCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScFSSTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScFSS timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScFSSCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScSTTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScST timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScSTCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScGPSTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScGPS timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScGPSCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScAccelTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScAccel timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScAccelCtr = (long) (dt/DTSIM+0.5);

         fscanf(infile,"%s %lf %[^\n] %[\n]",response,&dt,junk,&newline);
         ScShakerTlmEnabled = DecodeString(response);
         if (dt < DTSIM) {
            printf("ScShaker timestep < DTSIM.  You'll want to fix that.");
            exit(1);
         }
         else MaxScShakerCtr = (long) (dt/DTSIM+0.5);

      }

      if (ScTlmEnabled) WriteScToCsv();
      if (ScBTlmEnabled) WriteScBToCsv();
      if (ScGTlmEnabled) WriteScGToCsv();
      if (ScWhlTlmEnabled) WriteScWhlToCsv();
      if (ScGyroTlmEnabled) WriteScGyroToCsv();
      if (ScMAGTlmEnabled) WriteScMAGToCsv();
      if (ScCSSTlmEnabled) WriteScCSSToCsv();
      if (ScFSSTlmEnabled) WriteScFSSToCsv();
      if (ScSTTlmEnabled) WriteScSTToCsv();
      if (ScGPSTlmEnabled) WriteScGPSToCsv();
      if (ScAccelTlmEnabled) WriteScAccelToCsv();
      if (ScShakerTlmEnabled) WriteScShakerToCsv();

}

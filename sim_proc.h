#ifndef SIM_PROC_H
#define SIM_PROC_H

#include <vector>
#include <map>
#include <list>

using namespace std;

typedef unsigned long long ull;
typedef unsigned long int  ul;
typedef unsigned short int us;

typedef struct proc_params{
    ul rob_size;
    ul iq_size;
    ul width;
}proc_params;

// Put additional data structures here as per your requirement

typedef struct timings{
  ul arrival; //instruction in particular stage entered in which cc
  ul duration; //cc spent in particular stage
} cc_count;

//Per stage clock cyle info
typedef struct ccTimingBoard{
   cc_count FE;
   cc_count DE;
   cc_count RN;
   cc_count RR;

   cc_count DI;
   cc_count IQ;
   cc_count EX;
   cc_count WB;

   cc_count RT;

   void init()
   {
     FE.arrival = 0;
     FE.duration = 0;

     DE.arrival = 0;
     DE.duration = 0;

     RN.arrival = 0;
     RN.duration = 0;

     RR.arrival = 0;
     RR.duration = 0;

     DI.arrival = 0;
     DI.duration = 0;

     IQ.arrival = 0;
     IQ.duration = 0;

     EX.arrival = 0;
     EX.duration = 0;

     WB.arrival = 0;
     WB.duration = 0;

     RT.arrival = 0;
     RT.duration = 0;
   }

} cc_stat;

//for renamed source registers
typedef struct src_reg{
	short ID; 	//RMT index can be ARF or ROB tag , updated at RN stage
	bool val;   // updated in RN stage
	bool rdy;   // updated in RR stage
} src;

typedef struct renamed_reg{
  short dr;
  src s1;
  src s2;
} renamed;

typedef struct instruction_payload{
  ull     seqNo;

  ull     pc;
  us      opType;
  short   drID;
  short   sr1ID;
  short   sr2ID;

  cc_stat cc;       //clockcycle stats per stages
  us      computeCycles; //Operation Cycles Count, decrements in exe stage
  renamed rn;       //Instr info after rename stage

} payload;


typedef struct rob_struct{
  bool    val_ROB;
  ul      ind_ROB;

  short   dr_ROB;
  bool    rdy_ROB;
  payload instr;

  //fields not required :
  //ull value;
  //bool exception, mispredict;
  //ull   pc_ROB;
} rob_type;


//RMT index no indicates which register( r_, e.g r1, r5) is requested
typedef struct rmt_struct{

   bool valid;    //0: get reg from ARF. 1:reg is from ROB, rob_tag field is valid
   ul   rob_tag;

} rmt_type;


typedef struct issueQueue_struct{

  bool     valid_IQ; //0:iq slot is free/can overwrite, 1:occupied
  short    dr_IQ;    //always a ROB tag except value -1

  short    src1_IQ; //always a ROB tag except value -1
  bool     src1_rdy_IQ;

  short    src2_IQ;  //always a ROB tag except value -1
  bool     src2_rdy_IQ;

  payload  instr;
  bool     iq_entered;
  bool     exe_finished;
  bool     exe_started;

} iq_type;


//pipeline registers to keep current instrunction bundles
typedef struct pipeline_reg{

  std::vector<payload> DE;
  std::vector<payload> RN;
  std::vector<payload> RR;
  std::vector<payload> DI;
  std::vector<iq_type> WB;

} pipeReg;


class SuperscalarPipe{

  public:
      SuperscalarPipe(ul,ul,ul);
      virtual ~SuperscalarPipe(void);

      void FETCH(FILE*);
      void DECODE(void);
      void RENAME(void);
      void REGREAD(void);
      void DISPATCH(void);
      void ISSUE(void);
      void EXECUTE(void);
      void WRBACK(void);
      void RETIRE(void);
      bool advance_cycle(void);

      void finalResults(void);

  private:
      pipeReg pipe;

      ul WIDTH;
      ul EW_WIDTH;

      rob_type *ROB;
      ul ROB_SIZE;
      ul rob_HEAD, rob_TAIL;
      ul rob_vacancy_counter;

      rmt_type *RMT;
      //ARF is not required as values are not considered here

      iq_type *IQ;
      ul IQ_SIZE;
      ul iq_vacancy_count;

     std::list<iq_type> exe_list;
     ul exe_list_counter;

     void displayCommitedInst(void);
     void wakeup_from_front(int);

     //Additional
      bool rename_stalled;
      ul eof_width;

      void displayIQ(void);
      void displayRMT(void);
      void displayROB(void);
      void displayExecutionList(void);
  };


#endif

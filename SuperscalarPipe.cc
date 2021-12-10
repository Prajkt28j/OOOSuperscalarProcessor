#include<stdio.h>
#include "sim_proc.h"

using namespace std;

//To disable comments :
//#define PRINT_DBG_INFORMATION

#ifdef PRINT_DBG_INFORMATION
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

static ull CC_CURRENT = 0;
static ull INSTR_TOTAL = 0;
static bool ROB_FULL = false;
static bool IQ_EMPTY = true;
static bool ROB_EMPTY = true;


using namespace std;

SuperscalarPipe::SuperscalarPipe(ul t_width, ul t_iqSize, ul t_robSize)
{
    DBG_PRINTF("SuperscalarPipe Constructor");

    ROB       = NULL;
    IQ        = NULL;
    RMT       = NULL;

    WIDTH     = t_width;
    ROB_SIZE  = t_robSize;
    IQ_SIZE   = t_iqSize;
    EW_WIDTH  = 5 * WIDTH; //EXECUTION AND WRITE BACK units width

    rob_vacancy_counter = 0;
    iq_vacancy_count    = 0;
    exe_list_counter    = EW_WIDTH;

    rename_stalled      = false;
    eof_width           = 0;

    //Initialize dynamic scheduler units
    //1.Reorder Buffer
    if (ROB_SIZE)
    {
        ROB = new rob_type[ROB_SIZE + 1]; //accessing loc 1 to robsize
        rob_HEAD = rob_TAIL = 1;
        rob_vacancy_counter = ROB_SIZE;
        for (ull i = 1; i <= ROB_SIZE; ++i) {
            ROB[i].val_ROB      = false;
            ROB[i].instr.seqNo  = 0;
        }
    }
    //2. Issue Queue
    if (IQ_SIZE) {
        IQ = new iq_type[IQ_SIZE];
        iq_vacancy_count = IQ_SIZE;
        for (ull i = 0; i < IQ_SIZE; i++) { IQ[i].valid_IQ = false; }

    }

    //3.Rename Map Table
    if (ROB_SIZE) {
        RMT = new rmt_type[67]; //RMT and ARF both are of size 67 : r0-r66
        for (us i = 0; i < 67; i++) {
            RMT[i].valid = false;
            RMT[i].rob_tag = 0;
        }
    }

}

SuperscalarPipe::~SuperscalarPipe(void) {
    DBG_PRINTF("SuperscalarPipe Destructor");

    delete ROB;
    delete RMT;
    delete IQ;
}

// Advance_Cycle performs several functions.
//First, it advances the simulator cycle.
// Second, when it becomes known that the
// pipeline is empty AND the trace is depleted,
// the function returns “false” to terminate the loop.

bool SuperscalarPipe::advance_cycle(void) {
    ++CC_CURRENT; //Clock Cycle Count
   DBG_PRINTF("\n DE[%d]RN[%d]RR[%d]DI[%d]IQ_EMPTY[%d]exList[%d]WB[%d]ROB_EMPTY[%d]ROB_FULL[%d]H[%lu]T[%lu]\n\n",
          pipe.DE.empty(), pipe.RN.empty(), pipe.RR.empty(), pipe.DI.empty(), IQ_EMPTY, exe_list.empty(),
          pipe.WB.empty(), ROB_EMPTY, ROB_FULL, rob_HEAD, rob_TAIL);

    if (   pipe.DE.empty()
        && pipe.RN.empty()
        && pipe.RR.empty()
        && pipe.DI.empty()
        && IQ_EMPTY
        && (exe_list.empty())
        && pipe.WB.empty()
        && ROB_EMPTY)
    {
        return false;
    }

    DBG_PRINTF("\n\n ---------Next CC [%llu] starts--------", CC_CURRENT);

    return true;
}

// Do nothing if either (1) there are no  more instructions in the trace file or
// (2) DE is not empty (cannot accept a new decode bundle).
// If there are more instructions in the
// trace file and if DE is empty (can accept
// a new decode bundle), then fetch up to
// WIDTH instructions from the trace file
// into DE. Fewer than WIDTH instructions
// will be fetched only if the trace file
// has fewer than WIDTH instructions left.

void SuperscalarPipe::FETCH(FILE *readTraceFP) {

    if (!pipe.DE.empty()) return; //if decode is stalled, fetch is stalled too, proceed if decode pipe is empty

    payload instr;
    instr.cc.init();

    ull pc;
    int op_type;
    int dest, src1, src2;

    for (ul w = 0; w < WIDTH; w++) {
        if (fscanf(readTraceFP, "%llx %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF) {

           ++INSTR_TOTAL; //Instrunction Seq Number starts from 1

            DBG_PRINTF("\n ->FETCHED [%llu]inst[%llu]: %llx %d %d %d %d", CC_CURRENT, INSTR_TOTAL, pc, op_type, dest, src1, src2);

            instr.pc      = pc;
            instr.opType  = op_type;
            instr.drID    = dest;
            instr.sr1ID   = src1;
            instr.sr2ID   = src2;
            instr.seqNo   = INSTR_TOTAL;
            instr.cc.FE.arrival = CC_CURRENT;

            //Add instrunction into decode bundle
            pipe.DE.push_back(instr);

            pipe.DE[w].cc.DE.arrival = CC_CURRENT + 1;
            pipe.DE[w].cc.FE.duration = pipe.DE[w].cc.DE.arrival - pipe.DE[w].cc.FE.arrival;

        } else {
            if (pipe.DE.size() != WIDTH)
                eof_width = pipe.DE.size(); //noted last fetch instru count
                DBG_PRINTF("\n Trace File end reached eof_width[%lu]\n", eof_width);
            break;
        }
        DBG_PRINTF("\n Current INSTR COUNT--> %llu", INSTR_TOTAL);
    }
}

// If DE contains a decode bundle:
// If RN is not empty (cannot accept a new
// rename bundle), then do nothing.
// If RN is empty (can accept a new rename
// bundle), then advance the decode bundle
// from DE to RN.

void SuperscalarPipe::DECODE() {

    DBG_PRINTF("\n ->DECODE::start DE.size[%lu], RN.size[%lu]", pipe.DE.size(), pipe.RN.size());

    if (pipe.DE.empty())return; //no inst in DE pipeReg

    if (!pipe.RN.empty()) return;

    //proceed if reg rename is empty, ready to accept new instr
    if (pipe.RN.empty()) {

        for (ul w = 0; w < pipe.DE.size(); w++) {
            switch (pipe.DE[w].opType) {
                case 0:
                    pipe.DE[w].computeCycles = 1;
                    break;
                case 1:
                    pipe.DE[w].computeCycles = 2;
                    break;
                case 2:
                    pipe.DE[w].computeCycles = 5;
                    break;
            }

            //insert decode bundle into rename pipe
            pipe.RN.push_back(pipe.DE[w]);

            //Update clock cycle info for current and next pipeline Reg
            pipe.RN[w].cc.RN.arrival = CC_CURRENT + 1;
            pipe.RN[w].cc.DE.duration = pipe.RN[w].cc.RN.arrival - pipe.RN[w].cc.DE.arrival;

            DBG_PRINTF("\nPushed to RN");
        }

       //Empty Decode pipe for next instrunction bundle
        pipe.DE.clear();
    }
    DBG_PRINTF("\n DECODE::end DEsize[%lu], RN.size[%lu]", pipe.DE.size(), pipe.RN.size());
}

// If RN contains a rename bundle:
// If either RR is not empty (cannot accept
// a new register-read bundle) or the ROB
// does not have enough free entries to
// accept the entire rename bundle, then do
// nothing.
// If RR is empty (can accept a new
// register-read bundle) and the ROB has
// enough free entries to accept the entire
// rename bundle, then process (see below)
// the rename bundle and advance it from
// RN to RR.
// Apply your learning from the class
// lectures/notes on the steps for renaming:
// (1) allocate an entry in the ROB for the
// instruction, (2) rename its source
// registers, and (3) rename its destination
// register (if it has one). Note that the
// rename bundle must be renamed in program
// order (fortunately the instructions in
// the rename bundle are in program order).

void SuperscalarPipe::RENAME() {
    DBG_PRINTF("\n ->RENAME::start  rob_vacancy_counter[%lu] RN.size[%lu], RR.size[%lu]", rob_vacancy_counter, pipe.RN.size(), pipe.RR.size());

    if (pipe.RN.empty())return; //no inst in RN pipeReg

    //RR cannot accept a new register-read bundle or ROB doesnt have free entries
    if (!pipe.RR.empty()) return;


    if (rob_vacancy_counter < pipe.RN.size()) {
        rename_stalled = true;
        return;
    } else {
        rename_stalled = false;
    }

    for (ul w = 0; w < pipe.RN.size(); w++) {
         DBG_PRINTF("\n rob_vacancy_counter[%lu] rob_TAIL[%lu] rob_HEAD[%lu]", rob_vacancy_counter, rob_TAIL, rob_HEAD);

        //(1) allocate an entry in the ROB for the instruction
        ROB[rob_TAIL].ind_ROB = rob_TAIL; //additional
        ROB[rob_TAIL].val_ROB = true;

        ROB[rob_TAIL].dr_ROB  = pipe.RN[w].drID;
        ROB[rob_TAIL].rdy_ROB = false;
        pipe.RN[w].rn.dr      = rob_TAIL; //1.1.renamed dst reg to rob tag
        ROB[rob_TAIL].instr   = pipe.RN[w];

        DBG_PRINTF("\n New ROB entry--> TAIL[%lu]ROB_ID[%lu]drTag[%d]rdy[%d]seqNo[%llu]", rob_TAIL, ROB[rob_TAIL].ind_ROB , ROB[rob_TAIL].dr_ROB, ROB[rob_TAIL].rdy_ROB, ROB[rob_TAIL].instr.seqNo);

        //1.2)incr tail , decr vacancy_counter
        if (rob_vacancy_counter) { --rob_vacancy_counter; }

        if (rob_TAIL == ROB_SIZE){ rob_TAIL = 1; }
        else ++rob_TAIL;

        //(2) rename its source registers, check in RMT
         DBG_PRINTF("\n Original src1[%d]scr2[%d]", pipe.RN[w].sr1ID, pipe.RN[w].sr2ID);

        //rename scr1
        if (pipe.RN[w].sr1ID != -1) {
            if (RMT[pipe.RN[w].sr1ID].valid) { pipe.RN[w].rn.s1.ID = RMT[pipe.RN[w].sr1ID].rob_tag; }
            else {
                pipe.RN[w].rn.s1.ID = pipe.RN[w].sr1ID;
            }
            pipe.RN[w].rn.s1.val  = RMT[pipe.RN[w].sr1ID].valid;
        } else {
            pipe.RN[w].rn.s1.ID   = pipe.RN[w].sr1ID;
            pipe.RN[w].rn.s1.val  = false;
        }
        //rename scr2
        if (pipe.RN[w].sr2ID != -1) {
            if (RMT[pipe.RN[w].sr2ID].valid) { pipe.RN[w].rn.s2.ID = RMT[pipe.RN[w].sr2ID].rob_tag; }
            else {
                pipe.RN[w].rn.s2.ID = pipe.RN[w].sr2ID;
            }
            pipe.RN[w].rn.s2.val  = RMT[pipe.RN[w].sr2ID].valid;
        } else {
            pipe.RN[w].rn.s2.ID   = pipe.RN[w].sr2ID;
            pipe.RN[w].rn.s2.val  = false;
        }

        //(3) renamed destination register (if it has one) in RMT
        if (pipe.RN[w].drID != -1) {
            RMT[pipe.RN[w].drID].rob_tag  = pipe.RN[w].rn.dr;
            RMT[pipe.RN[w].drID].valid    = true;
        }
        DBG_PRINTF("\n -->RMT updated SeqNo[%llu] drID[%d]val[%d]tag[%lu]", pipe.RN[w].seqNo, pipe.RN[w].drID, RMT[pipe.RN[w].drID].valid, RMT[pipe.RN[w].drID].rob_tag);

        //insert renamed instrunction bundle into reg read pipe
        pipe.RR.push_back(pipe.RN[w]);

        //Update clock cycle info for current and next pipeline Reg
        pipe.RR[w].cc.RR.arrival = CC_CURRENT + 1;
        pipe.RR[w].cc.RN.duration = pipe.RR[w].cc.RR.arrival - pipe.RR[w].cc.RN.arrival;

    }

    if (rob_vacancy_counter == 0) ROB_FULL = true;
    else if (rob_vacancy_counter < ROB_SIZE) ROB_EMPTY = false;

    //Empty Rename pipe for next instrunction bundle
    pipe.RN.clear();

    for (ul w = 0; w < pipe.RR.size(); w++) {
        DBG_PRINTF("\n renamed seqNo[%llu]dr[%d]s1[%d]s2[%d]" ,  pipe.RR[w].seqNo,  pipe.RR[w].rn.dr , pipe.RR[w].rn.s1.ID , pipe.RR[w].rn.s2.ID );
    }

    DBG_PRINTF("\n RENAME::end RN.size[%lu], RR.size[%lu]", pipe.RN.size(), pipe.RR.size());
}

// If RR contains a register-read bundle:
// If DI is not empty (cannot accept a
// new dispatch bundle), then do nothing.
// If DI is empty (can accept a new dispatch
// bundle), then process (see below) the
// register-read bundle and advance it from
// RR to DI.
//
// Since values are not explicitly modeled,
// the sole purpose of the Register Read
// stage is to ascertain the readiness of
// the renamed source operands. Apply your
// learning from the class lectures/notes on
// this topic.
//
// Also take care that producers in their
// last cycle of execution wakeup dependent
// operands not just in the IQ, but also in
// two other stages including RegRead()
// (this is required to avoid deadlock). See
// Execute() description above.


void SuperscalarPipe::REGREAD() {
    DBG_PRINTF("\n ->REGREAD::start  RR.size[%lu], DI.size[%lu]", pipe.RR.size(), pipe.DI.size());

    if (pipe.RR.empty())return; //no inst in reg read pipe

    //DI cannot accept a new dispatch bundle , Dispatch is stalled as IQ is not empty
    if (!pipe.DI.empty()) return;

    for (ul w = 0; w < pipe.RR.size(); w++) {
        if (pipe.RR[w].rn.s1.val == 0){ pipe.RR[w].rn.s1.rdy = true; }
        else {
            pipe.RR[w].rn.s1.rdy = ROB[pipe.RR[w].rn.s1.ID].rdy_ROB;
        }
        if (pipe.RR[w].rn.s2.val == 0){ pipe.RR[w].rn.s2.rdy = true; }
        else {
            pipe.RR[w].rn.s2.rdy = ROB[pipe.RR[w].rn.s2.ID].rdy_ROB;
        }

        //insert regRead instrunctions with updated ready bit infor into dispatch pipe
        pipe.DI.push_back(pipe.RR[w]);

        //Update clock cycle info for current and next pipeline Reg
        pipe.DI[w].cc.DI.arrival = CC_CURRENT + 1;
        pipe.DI[w].cc.RR.duration = pipe.DI[w].cc.DI.arrival - pipe.DI[w].cc.RR.arrival;
    }

    //Empty RegRead pipe for next instrunction bundle
    pipe.RR.clear();

    for (ul w = 0; w < pipe.DI.size(); w++) {
         DBG_PRINTF("\n RegRead seqNo[%llu]dr[%d] s1[%d]v[%d]r[%d], s2[%d]v[%d]r[%d]" , pipe.DI[w].seqNo, pipe.DI[w].rn.dr , pipe.DI[w].rn.s1.ID, pipe.DI[w].rn.s1.val, pipe.DI[w].rn.s1.rdy, pipe.DI[w].rn.s2.ID, pipe.DI[w].rn.s2.val, pipe.DI[w].rn.s2.rdy );
    }

    DBG_PRINTF("\n  REGREAD::end  RR.size[%lu], DI.size[%lu]", pipe.RR.size(), pipe.DI.size());
}

// If DI contains a dispatch bundle:
// If the number of free IQ entries is less
// than the size of the dispatch bundle in
// DI, then do nothing. If the number of
// free IQ entries is greater than or equal
// to the size of the dispatch bundle in DI,
// then dispatch all instructions from DI to
// the IQ.

void SuperscalarPipe::DISPATCH() {
    DBG_PRINTF("\n ->DISPATCH::start  DI.size[%lu] iq_vacancy_count[%lu]", pipe.DI.size(), iq_vacancy_count);

    if (pipe.DI.empty())return; //no inst in DI pipeReg

    //return if IQ doesnt have enough free entries
    if (iq_vacancy_count < pipe.DI.size()) return;

    //1)Find free IQ indices for dispatch bundle size either of count WIDTH or of end of file instr bundle WIDTH
    ul index, i = 0;
    bool found = false;
    for (ul w = 0; w < pipe.DI.size(); ++w) {
        for (; i < IQ_SIZE; ++i) {
            if (IQ[i].valid_IQ == false) {
                index = i;        //noted for free spot
                i     = index;    //next interation starts from index+1 bcz of ++i
                found = true;
                break;
            }
        }
        if (found) {
            DBG_PRINTF("\n Dispatched to IQ inst seqNo[%llu] at [%lu]th loc", IQ[index].instr.seqNo, index);
            IQ[index].valid_IQ       = true;
            IQ[index].dr_IQ          = pipe.DI[w].rn.dr;
            IQ[index].src1_IQ        = pipe.DI[w].rn.s1.ID;
            IQ[index].src1_rdy_IQ    = pipe.DI[w].rn.s1.rdy;
            IQ[index].src2_IQ        = pipe.DI[w].rn.s2.ID;
            IQ[index].src2_rdy_IQ    = pipe.DI[w].rn.s2.rdy;
            IQ[index].instr          = pipe.DI[w];
            IQ[index].iq_entered     = true;

            --iq_vacancy_count;
            IQ_EMPTY = false; //as added one instr in IQ
            found = false; //reset for next iter
        }
    }
    //displayIQ();

    //Empty Dispatch pipe to accept next instrunction bundle
    pipe.DI.clear();

    DBG_PRINTF("\n  DISPATCH::end  DI.size[%lu]iq_vacancy_count[%lu]", pipe.DI.size(), iq_vacancy_count);
}

// Issue up to WIDTH oldest instructions from the IQ.
// (One approach to implement oldest-first issuing, is to make multiple
// passes through the IQ, each time finding  the next oldest ready instruction and then issuing it.
// One way to annotate the age of an instruction is to assign an incrementing sequence number to each
// instruction as it is fetched from the trace file.)
// To issue an instruction:
// 1) Remove the instruction from the IQ.
// 2) Add the instruction to the execute_list.
//    Set a timer for the instruction in the execute_list that to model its execution latency.

void SuperscalarPipe::ISSUE() {
     DBG_PRINTF("\n ->ISSUE::start iq_vacancy_count[%lu]IQ_SIZE[%lu]exe_list_counter[%lu]", iq_vacancy_count, IQ_SIZE, exe_list_counter);

    if (IQ_EMPTY) return;

    //If exe list doesn't have free entry, Don't issue
    if (!exe_list_counter) return;

    //Update clock cyle info
    for (ul w = 0; w < IQ_SIZE; ++w) {
        if (IQ[w].valid_IQ && IQ[w].iq_entered) {
            IQ[w].iq_entered            = false;
            IQ[w].instr.cc.IQ.arrival   = CC_CURRENT;
            IQ[w].instr.cc.DI.duration  = (IQ[w].instr.cc.IQ.arrival) - IQ[w].instr.cc.DI.arrival;
        }
    }

    //first find out all valid instrunctions and add into map with it's original instr seq no and IQ location ID
    std::map<ul, ul> valid_iq_list;
    if (IQ_SIZE > 0) {
        for (ul i = 0; i < IQ_SIZE; ++i) {
            if ((IQ[i].valid_IQ) && (IQ[i].src1_rdy_IQ) && (IQ[i].src2_rdy_IQ))
                valid_iq_list[IQ[i].instr.seqNo] = i;
        }
    }
    DBG_PRINTF("\n Ready to exe Instr Count[%lu]", valid_iq_list.size());

    std::map<ul, ul>::iterator itr;
    for (std::map<ul, ul>::iterator itr = valid_iq_list.begin(); itr != valid_iq_list.end(); ++itr) {
       DBG_PRINTF("\n SeqNo[%lu] IQ_ID[%lu] ", itr->first, itr->second);
     }

    //issue 'UPTO WIDTH oldest' instrunctions to exe unit, pick from valid_iq_list map
    if (!valid_iq_list.empty()) {

        iq_type local;
        ul count_w = 1;
        for (std::map<ul, ul>::iterator map_itr = valid_iq_list.begin(); map_itr != valid_iq_list.end(); map_itr++)
         {
            local = IQ[(map_itr->second)];

            IQ[map_itr->second].exe_finished = false;
            IQ[map_itr->second].exe_started = false;

            if (exe_list_counter > 0) {
                //Insert ready instr into execution list
                exe_list.push_back(local);
                --exe_list_counter;

                IQ[map_itr->second].valid_IQ = false;
                ++iq_vacancy_count; //IQ got one free entry

                if (count_w != WIDTH) ++count_w;
                else break; //Add maximum WIDTH no of instr
            }
        }
    }

    if (iq_vacancy_count == IQ_SIZE) {
        IQ_EMPTY = true;
        DBG_PRINTF("\n ----IQ EMPTIED");
        return;
    } else { IQ_EMPTY = false; }

    DBG_PRINTF("\n ->ISSUE::end iqVacCount[%lu]exeVacCounter[%lu]", iq_vacancy_count, exe_list_counter);
}


// From the execute_list, check for instructions that are finishing execution this cycle, and:
// 1) Remove the instruction from the execute_list.
// 2) Add the instruction to WB.
// 3) Wakeup dependent instructions (set their source operand ready flags) in the IQ, DI (the dispatch bundle), and RR (the register-read bundle).

void SuperscalarPipe::EXECUTE() {

    DBG_PRINTF("\n ->EXECUTE::start exSize[%lu]EW_WIDTH[%lu]", exe_list.size(), EW_WIDTH);

    if (exe_list.empty()) return;

    if (exe_list.size() > EW_WIDTH) return; // Excessive instr in execution list

    std::list<iq_type>::iterator itrEx = exe_list.begin();
    //Update clock cyle info
    for (; itrEx != exe_list.end(); ++itrEx) {
        if (itrEx->exe_started == false) {
            itrEx->exe_started          = true;
            itrEx->instr.cc.EX.arrival  = CC_CURRENT;
            itrEx->instr.cc.IQ.duration = (itrEx->instr.cc.EX.arrival) - (itrEx->instr.cc.IQ.arrival);
        }
    }

    //1)Execute instr by decrementing num of operation cycles count
    for (itrEx = exe_list.begin(); itrEx != exe_list.end(); ++itrEx) {
        DBG_PRINTF("\n Exe started CC[%llu] seqNo[%llu] opCc[%d] drROBTag[%d]",  CC_CURRENT, itrEx->instr.seqNo, itrEx->instr.computeCycles, itrEx->dr_IQ);

        if ((itrEx->instr.computeCycles > 0) && (!(itrEx->exe_finished))){ --(itrEx->instr.computeCycles); }

        if (itrEx->instr.computeCycles == 0) { itrEx->exe_finished = true; }
    }

    //2)Wakeup RR, DI, IQ with produced result reg tag
    for (itrEx = exe_list.begin(); itrEx != exe_list.end(); ++itrEx) {
        if (itrEx->exe_finished) {
            if (ROB[itrEx->dr_IQ].dr_ROB != -1) wakeup_from_front(itrEx->dr_IQ);
        }
    }

    //3) Add completed instruction to WB
    itrEx = exe_list.begin();
    while (itrEx != exe_list.end()) {
        if (itrEx->exe_finished) {
            pipe.WB.push_back((*itrEx));
            itrEx = exe_list.erase(itrEx);
            ++exe_list_counter;
        } else {
            ++itrEx;
        }
    }

    // displayExecutionList();
    DBG_PRINTF("\n ->EXECUTE::end exSize[%lu]", exe_list.size() );
}

// Process the writeback bundle in WB:
// For each instruction in WB, mark the
// instruction as “ready” in its entry in the ROB.

void SuperscalarPipe::WRBACK() {

    DBG_PRINTF("\n ->WRBACK::start   pipe.WB.Size[%lu]", pipe.WB.size() );

    if (pipe.WB.empty()) return;

    if (pipe.WB.size() > EW_WIDTH) return; //MISMATCH in WB size

    //Update clock cycles info for wb & RETIRE stage
    for (ul w = 0; w < pipe.WB.size(); ++w) {
        pipe.WB[w].instr.cc.WB.arrival = CC_CURRENT;
        pipe.WB[w].instr.cc.EX.duration = (pipe.WB[w].instr.cc.WB.arrival) - pipe.WB[w].instr.cc.EX.arrival;

        pipe.WB[w].instr.cc.RT.arrival = CC_CURRENT + 1;
        pipe.WB[w].instr.cc.WB.duration = (pipe.WB[w].instr.cc.RT.arrival) - pipe.WB[w].instr.cc.WB.arrival;
    }

    for (ul w = 0; w < pipe.WB.size(); ++w)
    {
        ROB[pipe.WB[w].dr_IQ].rdy_ROB = true;
        ROB[pipe.WB[w].dr_IQ].instr   = pipe.WB[w].instr;

        DBG_PRINTF("\n --->READY to commit instr updated in ROB robID[%lu] seqNo[%llu] dr[%d]", ROB[pipe.WB[w].dr_IQ].ind_ROB, ROB[pipe.WB[w].dr_IQ].instr.seqNo, pipe.WB[w].dr_IQ );
        wakeup_from_front(pipe.WB[w].dr_IQ);
    }

   //Empty WriteBack pipe to accept next completed instrunction bundle
    pipe.WB.clear();

    DBG_PRINTF("\n ->WRBACK::end pipe.WB.Size[%lu]",   pipe.WB.size() );
}


// Retire up to WIDTH consecutive
// “ready” instructions from the head of the ROB.

void SuperscalarPipe::RETIRE() {
    DBG_PRINTF("\n ->RETIRE::start   ROB HEAD[%lu] TAIL[%lu]", rob_HEAD, rob_TAIL );

    for (ul w = 0; w < WIDTH; ++w) {
        if ((ROB[rob_HEAD].val_ROB) && (ROB[rob_HEAD].rdy_ROB == true)) {
             DBG_PRINTF("\n Commiting from H[%lu] inst[%llu] pre commit ROBVacCount[%lu]\n", rob_HEAD,  ROB[rob_HEAD].instr.seqNo, rob_vacancy_counter);

            if (ROB[rob_HEAD].dr_ROB != -1) {
                if ((RMT[ROB[rob_HEAD].dr_ROB].rob_tag) == ROB[rob_HEAD].ind_ROB)//imp solved 37 cc extra
                {
                    RMT[ROB[rob_HEAD].dr_ROB].valid   = false;  //committed to ARF
                    RMT[ROB[rob_HEAD].dr_ROB].rob_tag = 0;      //additional rob tag starts from 1
                }
            }
            //Update commited instr RT duration
            ROB[rob_HEAD].instr.cc.RT.duration = (CC_CURRENT + 1) - ROB[rob_HEAD].instr.cc.RT.arrival;

            //Print commited instr
            displayCommitedInst();

            ROB[rob_HEAD].val_ROB = false;
            ROB[rob_HEAD].instr.seqNo = 0;

            //if head reached end of ROB, set head back to 1
            if (ROB_SIZE == rob_HEAD) rob_HEAD = 0;

            ++rob_HEAD;
            ++rob_vacancy_counter; //ROB got one free entry
        }
    }

    DBG_PRINTF("\n-------------- post commit ROBVacCount[%lu] ------\n", rob_vacancy_counter);
    if (rob_vacancy_counter == ROB_SIZE) {
        ROB_EMPTY = true;
        rob_HEAD = rob_TAIL = 1;//reset head & tail
    }

    DBG_PRINTF("\n ->RETIRE::end   ROB HEAD[%lu] TAIL[%lu]", rob_HEAD, rob_TAIL );
}


void SuperscalarPipe::wakeup_from_front(int drROBTag) {

 DBG_PRINTF("\n === wakeup----> for tag [%d]  ", drROBTag);
 //displayIQ();

    //wakeup IQ
    for (ul w = 0; w < IQ_SIZE; ++w) {
        if (IQ[w].valid_IQ) {
            //src1
            if ((IQ[w].src1_IQ == drROBTag)
                && (IQ[w].src1_rdy_IQ == false)) {  DBG_PRINTF("\n wakeup in IQ seqNo[%llu] src1", IQ[w].instr.seqNo);
                IQ[w].src1_rdy_IQ = true;
            }
            //src2
            if ((IQ[w].src2_IQ == drROBTag)
                && (IQ[w].src2_rdy_IQ == false)) {   DBG_PRINTF("\n wakeup in IQ seqNo[%llu] src2", IQ[w].instr.seqNo);
                IQ[w].src2_rdy_IQ = true;
            }
        }
    }

    //wake up RR pipe
    for (ul w = 0; w < pipe.RR.size(); ++w) {  //wakeup RegRead Pipe
        //src1
        if ((pipe.RR[w].rn.s1.ID == drROBTag)
            && (pipe.RR[w].rn.s1.rdy == false)) {  DBG_PRINTF("\n wakeup in RR seqNo[%llu] src1", pipe.RR[w].seqNo);
            pipe.RR[w].rn.s1.rdy = true;
        }
        //src2
        if ((pipe.RR[w].rn.s2.ID == drROBTag)
            && (pipe.RR[w].rn.s2.rdy == false)) {  DBG_PRINTF("\n wakeup in RR seqNo[%llu] src2", pipe.RR[w].seqNo);
            pipe.RR[w].rn.s2.rdy = true;
        }
    }

    //wakeup Dispatch Pipe
    for (ul w = 0; w < pipe.DI.size(); ++w) {
        //src1
        if ((pipe.DI[w].rn.s1.ID == drROBTag)
            && (pipe.DI[w].rn.s1.rdy == false)) {  DBG_PRINTF("\n wakeup in DI seqNo[%llu] src1", pipe.RR[w].seqNo);
            pipe.DI[w].rn.s1.rdy = true;
        }
        //src2
        if ((pipe.DI[w].rn.s2.ID == drROBTag)
            && (pipe.DI[w].rn.s2.rdy == false)) {  DBG_PRINTF("\n wakeup in DI seqNo[%llu] src2", pipe.RR[w].seqNo);
            pipe.DI[w].rn.s2.rdy = true;
        }
    }
}


//Print all CC info of each instruction per stage
void SuperscalarPipe::displayCommitedInst() {
    payload instr = ROB[rob_HEAD].instr;
    printf("%llu fu{%d} src{%d,%d} dst{%d} FE{%lu,%lu} DE{%lu,%lu} RN{%lu,%lu} RR{%lu,%lu} DI{%lu,%lu} IS{%lu,%lu} EX{%lu,%lu} WB{%lu,%lu} RT{%lu,%lu}\n",
           (instr.seqNo - 1), instr.opType, instr.sr1ID, instr.sr2ID, instr.drID,
            //FETCH
           instr.cc.FE.arrival,
           instr.cc.FE.duration,
            //DECODE
           instr.cc.DE.arrival,
           instr.cc.DE.duration,
            //RENAME
           instr.cc.RN.arrival,
           instr.cc.RN.duration,
            //REG READ
           instr.cc.RR.arrival,
           instr.cc.RR.duration,
            //DISPATCH
           instr.cc.DI.arrival,
           instr.cc.DI.duration,
            //ISSUE
           instr.cc.IQ.arrival,
           instr.cc.IQ.duration,
            //EXECUTE
           instr.cc.EX.arrival,
           instr.cc.EX.duration,
            //WRITE BACK
           instr.cc.WB.arrival,
           instr.cc.WB.duration,
            //RETIRE
           instr.cc.RT.arrival,
           instr.cc.RT.duration);
}

void SuperscalarPipe::finalResults(void) {

    printf("# Dynamic Instruction Count    = %llu\n", INSTR_TOTAL);
    printf("# Cycles                       = %llu\n", CC_CURRENT);
    printf("# Instructions Per Cycle (IPC) = %.2f\n", float(INSTR_TOTAL) / (CC_CURRENT));

}





/******************************************* ADDITIONAL METHODS ********************************************/

void SuperscalarPipe::displayExecutionList(void) {
    DBG_PRINTF("\n    ----- ExecutionList ----");
    if (exe_list.empty()) return;
    std::list<iq_type>::iterator itrEx;
    for (itrEx = exe_list.begin(); itrEx != exe_list.end(); ++itrEx) {
        printf("\n seqNo[%llu] OpCycles[%d]", itrEx->instr.seqNo, itrEx->instr.computeCycles);
    }
}


void SuperscalarPipe::displayIQ(void) {
    DBG_PRINTF("\n    -----  IQ  ----  ");
    for (ul i = 0; i < IQ_SIZE; ++i) {
        if (IQ[i].valid_IQ) {
            DBG_PRINTF("\n ID[%lu]seqNo[%llu] dst[%d] s1.r[%d]s1[%d] s2.r[%d]s2[%d]", i, IQ[i].instr.seqNo, IQ[i].dr_IQ,
                   IQ[i].src1_rdy_IQ, IQ[i].src1_IQ, IQ[i].src2_rdy_IQ, IQ[i].src2_IQ);
        }
    }
}

void SuperscalarPipe::displayROB(void) {
    DBG_PRINTF("\n    -----  ROB ----  ");
    for (ul i = 0; i <= ROB_SIZE; ++i) {
        DBG_PRINTF("\n ROB ID[%lu] seqNo[%llu] drReg[%d] rdy[%d]", ROB[i].ind_ROB, ROB[i].instr.seqNo, ROB[i].dr_ROB, ROB[i].rdy_ROB);
    }
}

void SuperscalarPipe::displayRMT(void) {
    DBG_PRINTF("\n    -----  RMT  ----  ");
    for (ul i = 0; i < 67; i++) {
         DBG_PRINTF("\n ID[%lu]val[%d]tag[%lu]", i, RMT[i].valid,  RMT[i].rob_tag);
    }
}

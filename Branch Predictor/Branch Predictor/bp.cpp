
/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include "cmath"
#include <vector>
using namespace std;

const int processor_Address_size=32 ;

enum Share_type {// finite state machine for prediction , with 4 states {00-SNT , 01-WNT , 10-WT , 11-ST}
    no_share,
    using_share_lsb,
    using_share_mid
};

class FSM {
public:
    unsigned curr_State;
    unsigned default_State;

    FSM(unsigned default_State): curr_State(default_State), default_State(default_State) {} // fsm C-tor
    ~FSM()=default; // fsm d-tor
    void fsm_taken_update(bool taken)
    {
        if(taken)
        {
            if(curr_State < 3)
            {
                curr_State= curr_State+1;
            }
        }
        else {
            if(curr_State > 0)
            {
                curr_State= curr_State-1;
            }
        }
    }
    void re_init_state(unsigned default_state)
    {
        curr_State = default_State;
    }
    unsigned get_curr_State()
    {
        return curr_State;
    }
};

class BTB_ROW{	// the line have a pointer to vector<fsm>
public:

    unsigned historySize;
    uint32_t tag;
    uint32_t target;
    uint32_t row_history;		 // local history for every row in case of (globalHist==0)
    vector<FSM>* prediction_fsm; // local fsm for every row in case of (globaltable==0)

    BTB_ROW(int historySize,uint32_t tag, uint32_t target,uint32_t row_history)
    {
        historySize = historySize;
        target = target;
        tag = tag;
        row_history= row_history;
    }
    ~BTB_ROW(){}
    void re_init_fsm(unsigned default_state)
    {
        for(int i = 0; i < (int)pow(2,historySize); i++)
        {
            (*prediction_fsm)[i].default_State=default_state;
        }
    }
    void set_Fsm_vec(vector<FSM>* new_fsm)
    {
        this->prediction_fsm = new_fsm;
    }

};

class BTB_table {
public:
    vector<BTB_ROW> BTB_lines;   /// vector of btb_rows
    unsigned btbSize;			 /// brach target buffer table size
    int is_Shared;
    bool isGlobalHist;
    bool isGlobalTable;
    unsigned tagSize;
    int historySize;
    unsigned initial_fsmState;
    vector<FSM>* globalfsm;
    uint32_t global_History;
    int num_of_branchs;
    int num_of_flushes;

    BTB_table(unsigned btbSize,int is_Shared,bool isGlobalHist,bool isGlobalTable, unsigned tagSize, int historySize,
              unsigned initial_fsmState,uint32_t global_History=0,int num_of_branchs=0,int num_of_flushes=0);

    bool BTB_table_Pred(uint32_t pc, uint32_t *dst);


    void BTB_table_Update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);

    void BTB_table_Stats(SIM_stats *curStats);

    int calc_index(uint32_t pc, uint32_t historyReg);

    void insert_row(uint32_t pc, uint32_t tag, uint32_t target_pc, int tableIndex);

    ~BTB_table();
};




uint32_t extracts_tag(uint32_t pc, unsigned int tagSize, unsigned int btbSize) {

    // As detailed in the hw pdf, the tag begins from the bit log2[𝐵𝑇𝐵 𝑆𝑖𝑧𝑒]+2
    int tag_start = 2 + log2(btbSize);
    uint32_t tag = 0;
    for (unsigned int i = 0; i < tagSize && (i + tag_start) < processor_Address_size; i++) {
        //extracts the tag from the pc
        int tag_bit = (pc >> (i + tag_start)) & 1;
        tag |= (tag_bit << i);
    }
    return tag;
}


int extract_index(uint32_t pc, unsigned int btbSize) {
    // number of bits for the index
    int index_size = log2(btbSize);
    // least two bits is 00 so ignore them & then extract the index
    int index = (pc >> 2) & ((1 << index_size) - 1);
    return index;
}



void BHR_UPDATE(uint32_t *historyReg, bool input, unsigned int historySize) {
    // update the bhr for taken replace with 1 , for not taken replace with 0
    *historyReg = ((*historyReg << 1) | (input ? 1 : 0)) & ((1U << historySize) - 1);
}




int BTB_table::calc_index(uint32_t pc, uint32_t historyReg) {
    // // a mask of 0111...
    uint32_t historyMask = (1 << historySize) - 1;

    if (is_Shared == no_share) {
        return historyReg;
    } else {
        uint32_t tmp = 0;
        if (is_Shared == using_share_lsb) { //share_lsb is XOR the least significant bits of PC with historyReg

            tmp = (pc >> 2) ^ (historyReg & historyMask);
        } else if (is_Shared == using_share_mid) { //share_mid is XOR start from 16 bits of PC with historyReg

            tmp = (pc >> 16) ^ (historyReg & historyMask);
        }

        tmp &= (1 << historySize) - 1;  // Mask to the size of the history register bits

        return tmp;
    }
}

bool next_instruction(uint32_t pc, uint32_t *dst )
{
    *dst = pc + 4;
    return false;

}

void BTB_table::insert_row(uint32_t pc, uint32_t tag, uint32_t target_pc, int index) {
    BTB_lines[index].tag = tag;
    BTB_lines[index].target = target_pc;
    BTB_lines[index].row_history = 0;
    if (isGlobalTable==0) {
        for (int i = 0; i < pow(2, historySize); i++) {
            (*BTB_lines[index].prediction_fsm)[i].re_init_state(initial_fsmState);
        }
    }
}

//##############################################################################################//


BTB_table::BTB_table(unsigned btbSize,int is_Shared,bool isGlobalHist,bool isGlobalTable, unsigned tagSize, int historySize,
                     unsigned initial_fsmState,uint32_t global_History,int num_of_branchs,int num_of_flushes):
        BTB_lines(btbSize,BTB_ROW(historySize,-1,0,0)), // vector<BTB_ROW> C-TOR
        btbSize(btbSize),is_Shared(is_Shared),isGlobalHist(isGlobalHist),isGlobalTable(isGlobalTable),tagSize(tagSize),
        historySize(historySize),initial_fsmState(initial_fsmState),globalfsm(new vector<FSM>((int)pow(2,historySize), FSM(initial_fsmState))),
        global_History(0),num_of_branchs(0), num_of_flushes(0)
{

//    vector<FSM>* local_fsm = new vector<FSM>((int)pow(2,historySize), FSM(initial_fsmState));
    if(!isGlobalTable) {
        for (unsigned i = 0; i < btbSize; i++) {
            vector<FSM> *local_fsm = new vector<FSM>((int) pow(2, historySize), FSM(initial_fsmState));

            BTB_lines[i].set_Fsm_vec(local_fsm);
        }
    }
    //!!!!!!!!!!!!!to make sure of vector c-tor/./
    //vector<FSM>* globalfsm = new vector<FSM>((int)pow(2,historySize), FSM(initial_fsmState));

}


bool BTB_table::BTB_table_Pred(uint32_t pc, uint32_t *dst)//BTB_table_predict
{
    int row_index = extract_index(pc, btbSize);
    int gh_index = calc_index(pc, global_History); // calc index for global
    int lh_index = calc_index(pc, BTB_lines[row_index].row_history); // calc index for local
    uint32_t tag = extracts_tag(pc, tagSize, btbSize);
    if (BTB_lines[row_index].tag != tag) { return next_instruction(pc,dst);}

    if (isGlobalHist==1 && isGlobalTable==1) {
        if ((*globalfsm)[gh_index].curr_State == 3 || (*globalfsm)[gh_index].curr_State == 2) {
            *dst = BTB_lines[row_index].target;
            return true;
        } else { return next_instruction(pc,dst);}

    } else if (isGlobalHist==1 && isGlobalTable==0) {
        if ((*BTB_lines[row_index].prediction_fsm)[gh_index].curr_State == 3||
            (*BTB_lines[row_index].prediction_fsm)[gh_index].curr_State == 2)
        {
            *dst = BTB_lines[row_index].target;
            return true;
        } else { return next_instruction(pc,dst);}
    } else if (isGlobalHist==0 && isGlobalTable==1) {
        if ((*globalfsm)[lh_index].curr_State == 3 || (*globalfsm)[lh_index].curr_State == 2)
        {
            *dst = BTB_lines[row_index].target;
            return true;
        } else { return next_instruction(pc,dst);}
    } else {
        if ((*BTB_lines[row_index].prediction_fsm)[lh_index].curr_State == 3||
            (*BTB_lines[row_index].prediction_fsm)[lh_index].curr_State == 2) {
            *dst = BTB_lines[row_index].target;
            return true;
        } else { return next_instruction(pc,dst);}
    }
}


void BTB_table::BTB_table_Update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    num_of_branchs++;
    int index = extract_index(pc, btbSize);
    int gh_index = calc_index(pc, global_History); // calc index for global
    int lh_index = calc_index(pc, BTB_lines[index].row_history); // calc index for local
    uint32_t tag = extracts_tag(pc, tagSize, btbSize);

    bool prediction=false;
    if ((pred_dst == targetPc && taken == true) || (pred_dst != targetPc && taken == false)) {
        prediction = true;
    }
    if (prediction==false) num_of_flushes++;
    if (BTB_lines[index].tag == tag) { // already exist branch
        BTB_lines[index].target = targetPc;
        if (isGlobalTable==1 && isGlobalHist==1 ) {
            (*globalfsm)[gh_index].fsm_taken_update(taken);
            BHR_UPDATE(&global_History, taken, historySize);
        } else if (isGlobalTable==0 && isGlobalHist==1 ) {
            (*BTB_lines[index].prediction_fsm)[gh_index].fsm_taken_update(taken);
            BHR_UPDATE(&global_History, taken, historySize);
        } else if (isGlobalTable==1 && isGlobalHist==0 ) {
            (*globalfsm)[lh_index].fsm_taken_update(taken);
            BHR_UPDATE(&BTB_lines[index].row_history, taken, historySize);
        } else {
            (*BTB_lines[index].prediction_fsm)[lh_index].fsm_taken_update(taken);
            BHR_UPDATE(&BTB_lines[index].row_history, taken, historySize);
        }
    } else {
        insert_row(pc, tag, targetPc, index);
        BTB_lines[index].target = targetPc;
        int gh_index_new = calc_index(pc, global_History); // calc index for global
        int lh_index_new = calc_index(pc, BTB_lines[index].row_history); // calc index for local
        if (isGlobalTable==1 && isGlobalHist==1 ) {
            (*globalfsm)[gh_index_new].fsm_taken_update(taken);
            BHR_UPDATE(&global_History, taken, historySize);
        } else if (isGlobalTable==0 && isGlobalHist==1 ) {
            (*BTB_lines[index].prediction_fsm)[gh_index_new].fsm_taken_update(taken);
            BHR_UPDATE(&global_History, taken, historySize);
        } else if (isGlobalTable==1 && isGlobalHist==0 ) {
            (*globalfsm)[lh_index_new].fsm_taken_update(taken);
            BHR_UPDATE(&BTB_lines[index].row_history, taken, historySize);
        } else {
            (*BTB_lines[index].prediction_fsm)[lh_index_new].fsm_taken_update(taken);
            BHR_UPDATE(&BTB_lines[index].row_history, taken, historySize);
        }
    }
}

void BTB_table::BTB_table_Stats(SIM_stats *curStats) {
    curStats->br_num = num_of_branchs;
    curStats->flush_num = num_of_flushes;
    int history_size;
    int num_of_machines;
    if (isGlobalHist==1) {
        history_size = historySize;
    } else {
        history_size = historySize * (btbSize);
    }
    if (isGlobalTable==1) {
        num_of_machines = pow(2, historySize);
    } else {
        num_of_machines = pow(2, historySize) * btbSize;
    }
    curStats->size = btbSize * (tagSize + 30 + 1) + history_size + num_of_machines * 2;

}


BTB_table* new_BTB_table = nullptr;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared)
{
    new_BTB_table = new BTB_table(btbSize,Shared,isGlobalHist,isGlobalTable,tagSize,historySize,fsmState);
    if(new_BTB_table == nullptr)
    {
        return -1;
    }
    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

    return new_BTB_table->BTB_table_Pred(pc, dst);;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    new_BTB_table->BTB_table_Update(pc, targetPc, taken, pred_dst);

}

void BP_GetStats(SIM_stats *curStats){
    new_BTB_table->BTB_table_Stats(curStats);
}
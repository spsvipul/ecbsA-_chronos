/** $lic$
 * Copyright (C) 2014-2019 by Massachusetts Institute of Technology
 *
 * This file is part of the Chronos FPGA Acceleration Framework.
 *
 * Chronos is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this framework in your research, we request that you reference
 * the Chronos paper ("Chronos: Efficient Speculative Parallelism for
 * Accelerators", Abeydeera and Sanchez, ASPLOS-25, March 2020), and that
 * you send us a citation of your work.
 *
 * Chronos is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include "../include/chronos.h" 
//#define DEBUG 


// The location pointing to the base of each of the arrays
const int ADDR_BASE_DIST = 5 << 2;
const int ADDR_BASE_EDGE_OFFSET = 3 << 2;
const int ADDR_BASE_NEIGHBORS = 4 << 2;

uint32_t* dist;
uint32_t* edge_offset;
uint32_t* edge_neighbors;


struct state{
  uint32_t x;
  uint32_t y;
};


struct cameFrom{
  state neighbor;
  state curr_state;
  uint32_t gscore;
  uint32_t pad[11];  
};

/*
struct Node{
  uint32_t gscore;
  state prev_neigh;
};*/

#define N 100
#define obs_per 20 //define approximately what % of map is obstacles 
uint32_t mapp[N][N]; //ma contain 1 at place of obstacles
uint32_t visit[N][N]; //set already visited nodes to true
uint32_t is_enq[N][N]; //nodes already enqueued

cameFrom map_info[N][N]; //store previous state info  [0] gscore [1],[2] = neighbor  [3],[4] = curr_state
state start, goal;//start and end point of agent
uint32_t solution[2300][2]; //store solution
uint32_t done;//global done flag set true when solution found

uint32_t cnt;


typedef unsigned int uint;
#define VISIT_NODE_TASK  0
//#define HELLO_WORLD  1
#define LLS_START  1
#define SAVE_SOL  2

uint32_t* debug;
#ifdef DEBUG

#define DEBUG_AREA_RANGE  420  // need to be changed , depending your input data.
void set_log( uint val  )
{
  int p_gv = debug[0] +1 ;// get the incremental pointer for writing debugging log
  //if(cur_gv >= get_global_size() ){ // in a case of pointer is out of the range.
  //  return;
 // }
   if( p_gv > DEBUG_AREA_RANGE  ) return;  // out of range
 
   debug[p_gv] = val;
   debug[0] ++;
  return;
}
#endif
float sqrt7(float x){
   unsigned int i = *(unsigned int*) &x; 
   // adjust bias
   i  += 127 << 23;
   // approximation of square root
   i >>= 1; 
   return *(float*) &i;
}

uint32_t eucl_dist(state curr){
  float dist_sq = ((goal.x - curr.x)*(goal.x - curr.x)) + ((goal.y - curr.y)*(goal.y - curr.y));
  uint32_t sqrt  = sqrt7(dist_sq);
  return sqrt;
}


bool is_valid_neighbor(uint32_t x ,uint32_t y){
  return x >= 0 && x < N && y >= 0 && y < N && mapp[x][y] ==0;
}

uint32_t f_heur(state curr){
  int temp1,temp2;
  temp1 = goal.x - curr.x;
  temp2 = goal.y - curr.y;
  if (temp1 < 0) temp1 = -temp1;
  if (temp2 < 0) temp2 = -temp2;
  
  return temp1+temp2;
  
}

void save_sol(uint ts, uint object, uint32_t x , uint32_t y ,  uint32_t gscore ){
  state temp;
  temp.x = x;
  temp.y = y;    
  //logic to write solution to solution array 
  uint32_t i = gscore +1;
  for(int j=0;j< i ; j++){
    solution[j][0] = temp.x;
    solution[j][1] = temp.y;  
    //swarm::info("x : %i , y: %i" , solution[j][0], solution[j][1]);
    temp = map_info[temp.x][temp.y].curr_state;
    
    continue; 
  }   
}



void lls_start(uint ts, uint object, uint32_t x , uint32_t y ,  uint32_t gscore  ) {

    if (done == 1) return;
    state curr_state;
    curr_state.x=x;
    curr_state.y=y;
    
    
    
    //set_log(goal.x);
    //set_log(goal.y);

    if (curr_state.x == goal.x && curr_state.y == goal.y){
      //found solution
      //set_log(curr_state.x);
      //set_log(curr_state.y);
      undo_log_write(&done, done);
      done=1;
      state temp;
      temp = curr_state;
      //return; 
            
      //logic to write solution to solution array
      uint32_t i = gscore +1;
      //set_log(gscore);
      //set_log(temp.y);     
      
      for(int j=0;j< i ; j++){
        
        solution[j][0] = temp.x;
        solution[j][1] = temp.y;
        uint32_t logvar;
        //logvar = (100* temp.x) + temp.y;
        //set_log(logvar); 
        //set_log(temp.x); 
        //set_log(temp.y);  
        temp = map_info[temp.x][temp.y].curr_state;
        
        //continue;
      }
      #ifdef DEBUG 
      set_log(i); 
      set_log(9); 
      set_log(cnt);
      set_log(solution[0][0]); 
      set_log(solution[0][1]);
      set_log(solution[i-1][0]); 
      set_log(solution[i-1][1]);      
      #endif
    }
    
    
    
    if (visit[curr_state.x][curr_state.y]==0){
      undo_log_write(&visit[curr_state.x][curr_state.y], visit[curr_state.x][curr_state.y]);
      visit[curr_state.x][curr_state.y]=1;
      
      if (is_valid_neighbor(curr_state.x +1 , curr_state.y)){    
        state next_state;
        next_state.x= curr_state.x +1;
        next_state.y= curr_state.y;                   
        if (is_enq[next_state.x][next_state.y]==0){ 
                  
          //heuristic calculation
          uint32_t tentative_gScore = gscore + 1;
          uint32_t fScore = tentative_gScore + f_heur(next_state); //focal heuristiccalculation
          //uint32_t ts = 20*fScore + (100- tentative_gScore); 
          //uint32_t nts = fScore ;
          uint32_t nts = tentative_gScore + eucl_dist(next_state)   ;
          if (nts< ts) nts = ts;
          
          //undo log write here ??
          map_info[next_state.x][next_state.y].gscore = tentative_gScore;
          map_info[next_state.x][next_state.y].neighbor = next_state;
          map_info[next_state.x][next_state.y].curr_state = curr_state;
        
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          //swarm::info("lls in : curr.x: %i , curr.y: %i , next.x : %i , next.y : %i ,  tent_gscore: %i", curr_state.x,curr_state.y,next_state.x,next_state.y, tentative_gScore);
          is_enq[next_state.x][next_state.y]=1;
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          enq_task_arg3(LLS_START, nts,(uint32_t) &visit[next_state.x][next_state.y], next_state.x,next_state.y,  tentative_gScore  );
        } 
      }
      
      if (is_valid_neighbor(curr_state.x - 1 , curr_state.y)){                                     
        state next_state;
        next_state.x= curr_state.x -1;
        next_state.y= curr_state.y;
          
        if (is_enq[next_state.x][next_state.y]==0){ 
          
          //heuristic calculation
          uint32_t tentative_gScore = gscore + 1;
          uint32_t fScore = tentative_gScore + f_heur(next_state); //focal heuristiccalculation
          //uint32_t ts = 20*fScore + (100-tentative_gScore);  
          uint32_t nts = fScore ;
          //uint32_t nts = tentative_gScore + eucl_dist(next_state)   ;
          //if (nts< ts) nts = ts;
          
          //undo log write here ??
          map_info[next_state.x][next_state.y].gscore = tentative_gScore;
          map_info[next_state.x][next_state.y].neighbor = next_state;
          map_info[next_state.x][next_state.y].curr_state = curr_state; 
          
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          //swarm::info("lls in : curr.x: %i , curr.y: %i , next.x : %i , next.y : %i ,  tent_gscore: %i", curr_state.x,curr_state.y,next_state.x,next_state.y, tentative_gScore);
          is_enq[next_state.x][next_state.y]=1;
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          enq_task_arg3(LLS_START, nts,(uint32_t) &visit[next_state.x][next_state.y] , next_state.x,next_state.y,  tentative_gScore  );
        }
      }
      
      if (is_valid_neighbor(curr_state.x  , curr_state.y + 1)){     
        state next_state;
        next_state.x= curr_state.x;
        next_state.y= curr_state.y + 1;
          
        if (is_enq[next_state.x][next_state.y]==0){ 
          
          //heuristic calculation
          uint32_t tentative_gScore = gscore + 1;
          uint32_t fScore = tentative_gScore + f_heur(next_state); //focal heuristiccalculation
          //uint32_t ts = 20*fScore + (100-tentative_gScore); 
          uint32_t nts = fScore ;
          //uint32_t nts = tentative_gScore + eucl_dist(next_state)   ;
          //if (nts< ts) nts = ts;
          
          //undo log write here ??
          map_info[next_state.x][next_state.y].gscore = tentative_gScore;
          map_info[next_state.x][next_state.y].neighbor = next_state;
          map_info[next_state.x][next_state.y].curr_state = curr_state;

          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          //swarm::info("lls in : curr.x: %i , curr.y: %i , next.x : %i , next.y : %i ,  tent_gscore: %i", curr_state.x,curr_state.y,next_state.x,next_state.y, tentative_gScore);
          is_enq[next_state.x][next_state.y]=1;
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          enq_task_arg3(LLS_START, nts, (uint32_t) &visit[next_state.x][next_state.y] , next_state.x,next_state.y,  tentative_gScore  );
        }     
      }
      
      if (is_valid_neighbor(curr_state.x , curr_state.y - 1)){        
        state next_state;
        next_state.x= curr_state.x ;
        next_state.y= curr_state.y - 1;
        
        if (is_enq[next_state.x][next_state.y]==0){        
          
          //heuristic calculation
          uint32_t tentative_gScore = gscore + 1;
          uint32_t fScore = tentative_gScore + f_heur(next_state); //focal heuristiccalculation
          //uint32_t ts = 20*fScore + (100-tentative_gScore); 
          uint32_t nts = fScore ;
          //uint32_t nts = tentative_gScore + eucl_dist(next_state)   ;
          //if (nts< ts) nts = ts;
          //undo log write here ??
          map_info[next_state.x][next_state.y].gscore = tentative_gScore;
          map_info[next_state.x][next_state.y].neighbor = next_state;
          map_info[next_state.x][next_state.y].curr_state = curr_state;
        
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          //swarm::info("lls in : curr.x: %i , curr.y: %i , next.x : %i , next.y : %i ,  tent_gscore: %i", curr_state.x,curr_state.y,next_state.x,next_state.y, tentative_gScore);
          is_enq[next_state.x][next_state.y]=1;
          //swarm::info("visit node info %i: " ,is_enq[next_state.x][next_state.y] );
          enq_task_arg3(LLS_START, nts, (uint32_t) &visit[next_state.x][next_state.y] , next_state.x,next_state.y,  tentative_gScore  );//(&map_info[next_state.x][next_state.y])
        } 
      }
  }
}
uint32_t random_u32(uint32_t prev) {
    return prev*48271U;
}

 

/*
void hello_world(uint ts, uint object  ) {

    
    //uint32_t temp = (*(uint32_t*) vid) ; 
    uint32_t temp = sqrt7(79); //8
    set_log(temp);
    temp = sqrt7(0);
    set_log(temp);
    temp = sqrt7(1);
    set_log(temp);
    temp = sqrt7(81); //9
    set_log(temp);
    temp = sqrt7(102); //10
    set_log(temp);
    
}
*/

void visit_node_task(uint ts, uint write_addr, uint vid) { 
      
      //uint stt;
      //stt=28;
      /*
      state stt;
      stt.x =5;
      stt.y=7;
      uint size = sizeof(stt);
      //set_log(size);
      goal.x=N-2;
      goal.y=N-2;
      
      enq_task_arg0(HELLO_WORLD, 0, (uint32_t) &dist[0]);
      return;  
      */
       
       cnt=0;
       uint32_t seed = 134775813U;   
       uint32_t rnum = random_u32(seed);
       for(int i=0;i<N;i++){ 
         for(int j=0;j<N;j++){
           
           if((rnum%100) < obs_per){
             mapp[i][j]=1;
             cnt= cnt+1;
           }
           rnum = random_u32(rnum);
         }
       }
       
       //set_log(cnt);
       //return;
       start.x=0;
       start.y=0;
       goal.x=N-2;
       goal.y=N-2;
       
       mapp[start.x][start.y]=0;
       mapp[goal.x][goal.y]=0;
       
       map_info[start.x][start.y].gscore = 0;
       map_info[start.x][start.y].neighbor = start;
       map_info[start.x][start.y].curr_state = start;
       
       
      enq_task_arg3(LLS_START, 0, (uint32_t) 0, start.x,start.y,  0 );
      
      
}


int main() {
  
   chronos_init();
   
   
   // Dereference the pointers to array base addresses.
   // ( The '<<2' is because graph_gen writes the word number, not the byte)
   //dist = (uint32_t*) ((*(uint32_t *) (ADDR_BASE_DIST))<<2) ;
   debug = (uint32_t*) ((*(uint32_t *) (ADDR_BASE_DIST))<<2) ;// this testcode temporally uses DIST area for debugging.
   debug[0]=0;
   /*edge_offset  =(uint32_t*) ((*(int *)(ADDR_BASE_EDGE_OFFSET))<<2) ;
   edge_neighbors  =(uint32_t*) ((*(int *)(ADDR_BASE_NEIGHBORS))<<2) ;

   set_cd_range(dist, edge_offset);*/

   while (1) {
      uint ttype, ts, object, vid, i,j;
      state stt;   
      deq_task_arg3(&ttype, &ts, &object, &vid, &i, &j);
      //deq_task_argst(&ttype, &ts, &object,&vid , &stt);
      
      switch(ttype){
          case VISIT_NODE_TASK:            
              visit_node_task(ts, object, vid);
              break;
          /*
          case HELLO_WORLD:
              hello_world(ts , object);
              break; 
          */
          case LLS_START:
              lls_start(ts , object, vid, i,j);
              break;
              
          case SAVE_SOL:
              save_sol(ts , object, vid, i,j);
              break;
          
          default:
              break;
      }

      finish_task();
   }
}


 

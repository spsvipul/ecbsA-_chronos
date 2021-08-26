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

// The location pointing to the base of each of the arrays
const int ADDR_BASE_DIST = 5 << 2;
const int ADDR_BASE_EDGE_OFFSET = 3 << 2;
const int ADDR_BASE_NEIGHBORS = 4 << 2;

uint32_t* dist;
uint32_t* edge_offset;
uint32_t* edge_neighbors;


#define VISIT_NODE_TASK  0
#define HELLO_WORLD  1
int array[20];

uint32_t* debug;
#define DEBUG_AREA_RANGE  16  // need to be changed , depending your input data.
void set_log( uint val  )
{
  int p_gv = debug[0];// get the incremental pointer for writing debugging log
  //if(cur_gv >= get_global_size() ){ // in a case of pointer is out of the range.
  //  return;
 // }
   if( p_gv > DEBUG_AREA_RANGE  ) return;  // out of range
 
   debug[p_gv] = val;
   debug[0] ++;
  return;
}

void hello_world(uint ts, uint object, uint vid ) {
    if (ts > 5){ return;
    }
    
    /*for (uint i=0;i<12;i++){
      set_log(i);
      array[i] = i;
    
    }*/
    set_log(vid);
        
    enq_task_arg1(HELLO_WORLD, ts +1 , (uint32_t) &dist[0], vid+1 );
    
  
    //int printf("Hello world");

}



void visit_node_task(uint ts, uint write_addr, uint vid) { 

      
      enq_task_arg1(HELLO_WORLD, 0, (uint32_t) &dist[0], vid);
      /*unsigned int cur_dist = (unsigned int) dist[vid];
      if (cur_dist <= ts) {
         return;
      }

      undo_log_write(&dist[vid], cur_dist);
      dist[vid] = ts;
      for (int i = edge_offset[vid]; i < edge_offset[vid+1]; i++) {
         int neighbor = edge_neighbors[i*2];
         int weight = edge_neighbors[i*2+1];


         enq_task_arg1(VISIT_NODE_TASK, ts + weight, (uint32_t) &dist[neighbor], neighbor);
      }*/
      
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
      uint ttype, ts, object, vid, i;
      deq_task_arg1(&ttype, &ts, &object, &vid);
      switch(ttype){
          case VISIT_NODE_TASK:
              visit_node_task(ts, object, vid);
              break;
          case HELLO_WORLD:
              hello_world(ts , object,vid);
              break;
          default:
              break;
      }

      finish_task();
   }
}


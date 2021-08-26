export PATH="/home/centos/.local/xPacks/@xpack-dev-tools/riscv-none-embed-gcc/10.1.0-1.1.1/.content/bin:$PATH"
#cd /home/centos/chronos/ 
#source aws_setup.sh
cd /home/centos/chronos/cl_chronos/riscv_code/sssp  
make clean
make 
cd /home/centos/chronos/cl_chronos/software/runtime/
cp /home/centos/chronos/cl_chronos/riscv_code/sssp/main.hex   ./main.sssp.hex
#make clean
#make
#echo 
sudo fpga-clear-local-image -S 0
#sudo fpga-clear-local-image -S 0

#sssp /w log 2tl8c
#sudo fpga-load-local-image -S 0 -I  agfi-08de86d70fa00ca1d
#sudo fpga-load-local-image -S 0 -I  agfi-08de86d70fa00ca1d
#sudo   ./test_chronos --n_tiles=2 sssp  grid_1000x1000.sssp   

#sssp /wo log 2tl8c
#sudo fpga-load-local-image -S 0 -I  agfi-0744ee8209f562c11
#sudo fpga-load-local-image -S 0 -I  agfi-0744ee8209f562c11
#sudo   ./test_chronos --n_tiles=1 sssp  grid_1000x1000.sssp  

#sssp /wo log 4tl8c
#sudo fpga-load-local-image -S 0 -I  agfi-02a7caac8460fb8a7
#sudo fpga-load-local-image -S 0 -I  agfi-02a7caac8460fb8a7
#sudo   ./test_chronos --n_tiles=2 sssp  grid_100x100.sssp

#4tile 4 core riscv
#sudo fpga-load-local-image -S 0 -I  agfi-0c804117f16bbd024
#sudo fpga-load-local-image -S 0 -I  agfi-0c804117f16bbd024

#2tile 4 core riscv
sudo fpga-load-local-image -S 0 -I  agfi-0f4e9eeb6debeef34
sudo fpga-load-local-image -S 0 -I  agfi-0f4e9eeb6debeef34  


#4tile 1 core riscv
#sudo fpga-load-local-image -S 0 -I agfi-08cebb03054c07bb6
#sudo fpga-load-local-image -S 0 -I agfi-08cebb03054c07bb6


#sudo   ./test_chronos --n_tiles=2 --n_threads=4 sssp  grid_21x21.sssp   main.sssp.hex
#sudo   ./test_chronos --n_tiles=1 --n_threads=4  sssp  grid_21x21.sssp   main.sssp.hex 
#sudo   ./test_chronos --n_tiles=1 --n_threads=2  sssp  grid_21x21.sssp   main.sssp.hex 
#sudo   ./test_chronos --n_tiles=1 --n_threads=1  sssp  grid_21x21.sssp   main.sssp.hex

sudo   ./test_chronos --n_tiles=2 sssp  grid_4x4.sssp   main.sssp.hex 
#sudo   ./test_chronos --n_tiles=1 --n_threads=4  sssp  grid_4x4.sssp   main.sssp.hex 
#sudo   ./test_chronos --n_tiles=1 --n_threads=2  sssp  grid_4x4.sssp    main.sssp.hex 
#sudo   ./test_chronos --n_tiles=3 --n_threads=1 --map=3  sssp  grid_4x4.sssp    main.sssp.hex

cd /home/centos/chronos/cl_chronos/riscv_code/sssp

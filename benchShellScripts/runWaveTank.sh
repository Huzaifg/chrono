./demo_FSI_WaveTankTest --ps_freq 1 --output true  
./demo_FSI_WaveTankTest --ps_freq 2 --output true  
./demo_FSI_WaveTankTest --ps_freq 4 --output true  
./demo_FSI_WaveTankTest --ps_freq 8 --output true  

./demo_FSI_WaveTankTest  --ps_freq 1 --output true  
./demo_FSI_WaveTankTest  --ps_freq 2 --output true  
./demo_FSI_WaveTankTest  --ps_freq 4 --output true  
./demo_FSI_WaveTankTest  --ps_freq 8 --output true  

## Now for benchmarking speed
mkdir -p WaveTankPerformance
./demo_FSI_WaveTankTest  --ps_freq 1 --output false --no_vis true --snapshots false  > WaveTankPerformance/1.log
./demo_FSI_WaveTankTest  --ps_freq 2 --output false --no_vis true --snapshots false  > WaveTankPerformance/2.log
./demo_FSI_WaveTankTest  --ps_freq 4 --output false --no_vis true --snapshots false  > WaveTankPerformance/4.log
./demo_FSI_WaveTankTest  --ps_freq 8 --output false --no_vis true --snapshots false  > WaveTankPerformance/8.log


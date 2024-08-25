./demo_FSI_SphereBounceTest --ps_freq 1 --output true --snapshots true 
./demo_FSI_SphereBounceTest --ps_freq 2 --output true --snapshots true 
./demo_FSI_SphereBounceTest --ps_freq 4 --output true --snapshots true 
./demo_FSI_SphereBounceTest --ps_freq 8 --output true --snapshots true 

./demo_FSI_SphereBounceTest  --ps_freq 1 --output true --snapshots true 
./demo_FSI_SphereBounceTest  --ps_freq 2 --output true --snapshots true 
./demo_FSI_SphereBounceTest  --ps_freq 4 --output true --snapshots true 
./demo_FSI_SphereBounceTest  --ps_freq 8 --output true --snapshots true 

## Now for benchmarking speed
mkdir -p SphereBouncePerformance
./demo_FSI_SphereBounceTest  --ps_freq 1 --output false --no_vis true --snapshots false  > SphereBouncePerformance/1.log
./demo_FSI_SphereBounceTest  --ps_freq 2 --output false --no_vis true --snapshots false  > SphereBouncePerformance/2.log
./demo_FSI_SphereBounceTest  --ps_freq 4 --output false --no_vis true --snapshots false  > SphereBouncePerformance/4.log
./demo_FSI_SphereBounceTest  --ps_freq 8 --output false --no_vis true --snapshots false  > SphereBouncePerformance/8.log


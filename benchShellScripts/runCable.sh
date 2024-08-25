./demo_FSI_Flexible_CableTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Explicit.json --ps_freq 1 --output true  
./demo_FSI_Flexible_CableTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Explicit.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_CableTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Explicit.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_CableTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Explicit.json  --ps_freq 8 --output true

./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json  --ps_freq 1 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json  --ps_freq 8 --output true 

./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_I2SPH.json  --ps_freq 1 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_I2SPH.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_I2SPH.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_I2SPH.json  --ps_freq 8 --output true  

## Now for benchmarking speed
mkdir -p FlexibleCablePerformance
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json --ps_freq 1 --output false --no_vis true --snapshots false > FlexibleCablePerformance/1.log
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json --ps_freq 2 --output false --no_vis true --snapshots false > FlexibleCablePerformance/2.log
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json --ps_freq 4 --output false --no_vis true --snapshots false > FlexibleCablePerformance/4.log
./demo_FSI_Flexible_CableTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Cable_Granular.json --ps_freq 8 --output false --no_vis true --snapshots false > FlexibleCablePerformance/8.log


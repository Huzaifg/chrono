./demo_FSI_Flexible_Flat_PlateTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Explicit.json --ps_freq 1 --output true  
./demo_FSI_Flexible_Flat_PlateTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Explicit.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_Flat_PlateTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Explicit.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_Flat_PlateTest --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Explicit.json  --ps_freq 8 --output true  

./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json  --ps_freq 1 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json  --ps_freq 8 --output true  

./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_I2SPH.json  --ps_freq 1 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_I2SPH.json  --ps_freq 2 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_I2SPH.json  --ps_freq 4 --output true  
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_I2SPH.json  --ps_freq 8 --output true  

# Now for benchmarking speed
mkdir -p FlexibleFlatPlatePerformance
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json --ps_freq 1 --output false --no_vis true --snapshots false > FlexibleFlatPlatePerformance/1.log
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json --ps_freq 2 --output false --no_vis true --snapshots false > FlexibleFlatPlatePerformance/2.log
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json --ps_freq 4 --output false --no_vis true --snapshots false > FlexibleFlatPlatePerformance/4.log
./demo_FSI_Flexible_Flat_PlateTest  --inputJSON ../data/fsi/input_json/demo_FSI_Flexible_Flat_Plate_Granular.json --ps_freq 8 --output false --no_vis true --snapshots false > FlexibleFlatPlatePerformance/8.log

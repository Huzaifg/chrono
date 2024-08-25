./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Explicit.json --ps_freq 1 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Explicit.json --ps_freq 2 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Explicit.json --ps_freq 4 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Explicit.json --ps_freq 8 --output true --snapshots true 

./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 1 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 2 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 4 --output true --snapshots true 
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 8 --output true --snapshots true 

## Now for benchmarking speed
mkdir -p DamBreakPerformance
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 1 --output false --no_vis true --snapshots false  > DamBreakPerformance/1.log
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 2 --output false --no_vis true --snapshots false  > DamBreakPerformance/2.log
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 4 --output false --no_vis true --snapshots false  > DamBreakPerformance/4.log
./demo_FSI_DamBreakTest --inputJSON ../data/fsi/input_json/demo_FSI_DamBreak_Granular.json --ps_freq 8 --output false --no_vis true --snapshots false  > DamBreakPerformance/8.log


